import rospy
from sensor_msgs import msg 
# from robot import robot
from arm import arm as robot

import numpy as np
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from types import NoneType
from sensor_msgs.msg._JointState import JointState
from mtm import mtm 
import sensor_msgs
from sensor_msgs.msg import Joy
from geometry_msgs.msg._PoseStamped import PoseStamped
import std_msgs
from std_msgs.msg import Bool

# TODO:
#     

class ClutchControl:
    # General idea:
    # Clutch
    # Get mtmr forward kinematics
    # Move mtmr
    # Get forward kinematics again
    # get the vector that was traveled
    # map x,y,z to pan, tilt, insertion in the camera
    # When unclutched don't move the camera anymore
    
    class MODE:
        """
            simulation mode: Use the hardware for the master side, 
                            and simulation for the ECM
            hardware mode: Use hardware for both the master side,
                            and the ECM
        """
        simulation = "SIMULATION"
        hardware = "HARDWARE"
        
    def __init__(self, mode = MODE.simulation):        
        self.__mode__ = mode
        self.movement_scale = .2 
        self.joint_angles = [0,0,0,0]
        
        self.mtml_pos = [0,0,0]
        self.mtml_joint_angles = [0,0,0,0,0,0,0]
        
        self.mtmr_pos = [0,0,0]
        self.mtmr_joint_angles = [0,0,0,0,0,0,0]
        
        
        self.__init_nodes__()
        self.__spin__()
        
    def __init_nodes__(self):
        rospy.init_node('ecm_clutch_control')
        
        self.ecm_hw = robot("ECM")
        self.ecm_sim = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=10)
        self.ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        self.mtmr_robot = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
        self.mtml_robot = URDF.from_parameter_server('/dvrk_mtml/robot_description')
        self.ecm_kin = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[-1].name)
        self.mtmr_kin = KDLKinematics(self.mtmr_robot, self.mtmr_robot.links[0].name, self.mtmr_robot.links[-1].name)
        self.mtml_kin = KDLKinematics(self.mtml_robot, self.mtml_robot.links[0].name, self.mtml_robot.links[-1].name)
        
        self.mtml_orientation = mtm('MTML')
        self.mtmr_orientation = mtm('MTMR')
        
        self.mtml_psm2_orientation = rospy.Publisher('/dvrk/MTML_PSM2/lock_rotation', Bool, queue_size=1, latch=True )
        self.mtml_psm2_translation = rospy.Publisher('/dvrk/MTML_PSM2/lock_translation', Bool, queue_size=1, latch=True )
        
        self.mtmr_psm1_orientation = rospy.Publisher('/dvrk/MTMR_PSM1/lock_rotation', Bool, queue_size=1, latch=True )
        self.mtmr_psm1_translation = rospy.Publisher('/dvrk/MTMR_PSM1/lock_translation', Bool, queue_size=1, latch=True )
        
        if self.__mode__ == self.MODE.simulation:
            rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.ecm_cb)
        elif self.__mode__ == self.MODE.hardware:
            rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.ecm_cb)
            self.ecm_hw.home()
            self.ecm_hw.move_joint_list([0.0,0.0,0.0,0.0], interpolate=True)
        
        self.camera_clutch_pressed = False
        rospy.Subscriber('/dvrk/footpedals/camera_minus', Joy, self.camera_clutch_cb )
        self.mtml_starting_point = None
        
        rospy.Subscriber('/dvrk/MTML/position_cartesian_local_current', PoseStamped, self.mtml_cb)
        rospy.Subscriber('/dvrk/MTML/state_joint_current', JointState, self.mtml_joint_angles_cb)
        
        rospy.Subscriber('/dvrk/MTMR/position_cartesian_local_current', PoseStamped, self.mtmr_cb)
        rospy.Subscriber('/dvrk/MTMR/state_joint_current', JointState, self.mtmr_joint_angles_cb)
        
#         self.mtml_orientation.lock_orientation_as_is()
#         self.mtml_orientation.unlock_orientation()
        
    def __spin__(self):
        rospy.spin()

    def mtml_joint_angles_cb(self, msg):
        self.mtml_joint_angles = msg.position
    
    def mtmr_joint_angles_cb(self, msg):
        self.mtmr_joint_angles = msg.position
        
            
    def mtml_cb(self, msg):
#         msg.pose.position.x
        if self.camera_clutch_pressed:
            if type(self.mtml_starting_point) == NoneType:
                self.mtml_starting_point = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            current_position = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]) 
            movement_vector = current_position-self.mtml_starting_point
            self.move_mtm_centerpoints(self.mtml_pos, self.mtmr_pos)
#             print("movement_vector = {}, {}".format(movement_vector[0], movement_vector[1]))
            self.mtml_pos = current_position
            self.ecm_pan_tilt(movement_vector[0:2])
        else:
            self.center = self.joint_angles
            self.mtml_pos_before_clutch = msg.pose.position
    
    def mtmr_cb(self, msg):
        pass
        if self.camera_clutch_pressed:
            if type(self.mtml_starting_point) == NoneType:
                self.mtml_starting_point = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            current_position = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]) 
            movement_vector = current_position-self.mtml_starting_point
            self.mtmr_pos = current_position
#             self.ecm_pan_tilt(movement_vector[0:2])
        else:
#             self.center = self.joint_angles
            self.mtmr_pos_before_clutch = msg.pose.position

    # To be completed
    def move_mtm_centerpoints(self, mtml_pos, mtmr_pos):
        left = np.array([self.mtml_pos_before_clutch.x, self.mtml_pos_before_clutch.y, self.mtml_pos_before_clutch.z])
        right = np.array([self.mtmr_pos_before_clutch.x, self.mtmr_pos_before_clutch.y, self.mtmr_pos_before_clutch.z])
        mid = (left+right)/2.0
        
        if type(mtml_pos) != NoneType:
#             mtml_vect = np.array([mtml_pos.pos.position.x, mtml_pos.pos.position.y, mtml_pos.pos.position.z])
            
            ml_vector = mid-left
            mr_vector = right-mid
            
            new_mid = mtml_pos + ml_vector
            new_mtmr_position = new_mid + mr_vector
            
            mtmr_pose = self.mtmr_kin.forward(list(self.mtmr_joint_angles)[0:-1])
            print('ml_vector = ' + ml_vector.__str__())
            print('mr_vector = ' + mr_vector.__str__())
            print('new_mid = ' + new_mid.__str__())
            print('new_mtmr_position = ' + new_mtmr_position.__str__())
            
            new_mtmr_position = np.array(self.mtml_pos)-left + right
            print('new_mtmr_position = ' + new_mtmr_position.__str__())
            
            mtmr_pose[0:3, 3] = new_mtmr_position.reshape(3,1)
            mtmr_joint_angles = self.mtmr_kin.inverse(mtmr_pose)
            print('mtmr_pose = ' + mtmr_pose.__str__())
            print('mtmr_joint_angles = ' + mtmr_joint_angles.__str__())
             
#         
#         # distance between the two mtms before clutching
#         d = np.sqrt( (left[0]-right[0])**2 + (left[1]-right[1])**2 + (left[2]-right[2])**2)
#         
#         # Equation of the line that passes through both mtms
#         # t is the ratio of distance from the left mtm
#         x = lambda t: left[0] + right[0] * t
#         y = lambda t: left[1] + right[1] * t
#         z = lambda t: left[2] + right[2] * t
        
        
        
            
    def camera_clutch_cb(self, msg):
        if msg.buttons[0] == 1: # Camera clutch pressed
            self.camera_clutch_pressed = True
            self.mtml_starting_point = None
            self.mtml_orientation.lock_orientation_as_is()
            self.mtmr_orientation.lock_orientation_as_is()
            
            # Prevent the psms from moving when we're moving the camera
            self.mtml_psm2_orientation.publish(Bool(True))
            self.mtmr_psm1_orientation.publish(Bool(True))
            self.mtml_psm2_translation.publish(Bool(True))
            self.mtmr_psm1_translation.publish(Bool(True))
            
        else: # Camera clutch not pressed anymore
            self.camera_clutch_pressed = False
            self.center = self.joint_angles
            self.mtml_orientation.unlock_orientation()
            self.mtmr_orientation.unlock_orientation()
            
            # Return to teleop
            self.mtml_psm2_orientation.publish(Bool(False))
            self.mtmr_psm1_orientation.publish(Bool(False))
            self.mtml_psm2_translation.publish(Bool(False))
            self.mtmr_psm1_translation.publish(Bool(False))
                            
    def ecm_cb(self, msg):
        if self.__mode__ == self.MODE.simulation:
            self.joint_angles = msg.position[0:2] + msg.position[-2:]
        elif self.__mode__ == self.MODE.hardware:
            self.joint_angles = msg.position[0:2] + (0,0)
            
        if self.camera_clutch_pressed:
            if self.__mode__ == self.MODE.simulation:
                self.center = msg.position[0:2] + msg.position[-2:]
            elif self.__mode__ == self.MODE.hardware:
                self.center = msg.position[0:2] + (0,0)
            self.center_cart = np.array(self.ecm_kin.FK(self.center)[0])
            
    
    
    
    def ecm_pan_tilt(self, movement_vector):
        q = []
        q = self.joint_angles
        if q:
            newq = [q[0], q[1], .14, q[3]]
            ee = np.array(self.ecm_kin.FK(newq)[0])
            ee[0] = self.center_cart[0] + movement_vector[0] #* .01
            ee[1] = self.center_cart[1] + movement_vector[1] #* .01
             
            ee_inv = self.ecm_inverse(ee)
             
            if type(ee_inv) == NoneType:
                return
             
            new_joint_angles = [float(i) for i in ee_inv]
            self.move_ecm(new_joint_angles)
             
#             q = list(q)
#             q[0] = movement_vector[0] * self.movement_scale
#             q[1] = movement_vector[1] * self.movement_scale
#             q = [round(i,4) for i in q]
#             q = [i+j for i,j in zip(self.center, q)]
#             print(q)
#             self.move_ecm(q)
    
    # move ecm based on joint angles either in simulation or hardware
    def move_ecm(self, joint_angles):
        if self.__mode__ == self.MODE.simulation:
            msg = JointState()
            msg.position = joint_angles
            msg.name = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
            self.ecm_sim.publish(msg)            
        elif self.__mode__ == self.MODE.hardware:
            self.ecm_hw.move_joint_list(joint_angles, interpolate=False)
    
    def ecm_inverse(self, goal_pos):
        key_hole,_ = self.ecm_kin.FK([0,0,0,0])
        safe_angles = list(self.joint_angles)
        safe_angles[2] += 0.14
        
        ecm_pose = self.ecm_kin.forward(safe_angles)
        ab_vector = (goal_pos - key_hole)
        
        b, _ = self.ecm_kin.FK(safe_angles)
        
        ecm_current_direction = b - key_hole
        
        r = self.find_rotation_matrix_between_two_vectors(ecm_current_direction, ab_vector)
        m = np.sqrt(ab_vector[0]**2 + ab_vector[1]**2 + ab_vector[2]**2) # ab_vector's length
        
        # insertion joint length
        l = np.sqrt( (ecm_pose[0,3]-key_hole[0])**2 + (ecm_pose[1,3]-key_hole[1])**2 + (ecm_pose[2,3]-key_hole[2])**2)
        
        # Equation of the line that passes through the midpoint of the tools and the key hole
        x = lambda t: key_hole[0] + ab_vector[0] * t
        y = lambda t: key_hole[1] + ab_vector[1] * t
        z = lambda t: key_hole[2] + ab_vector[2] * t
        
        t = l/m
        
        new_ecm_position = np.array([x(t), y(t), z(t)]).reshape(3,1)
        
        ecm_pose[0:3,0:3] =  r* ecm_pose[0:3,0:3]  
        ecm_pose[0:3,3] = new_ecm_position
        
        
        new_joint_angles = self.joint_angles
        
        p = None
        try:
            p = self.ecm_kin.inverse(ecm_pose)
        except Exception as e:
            rospy.logerr('error')
        if type(p) != NoneType:  
            p[3] = 0
            p[2] -= 0.14
            new_joint_angles = p
        return new_joint_angles
    
    def find_rotation_matrix_between_two_vectors(self, a,b):
        a = np.array(a).reshape(1,3)[0].tolist()
        b = np.array(b).reshape(1,3)[0].tolist()
        
        vector_orig = a / np.linalg.norm(a)
        vector_fin = b / np.linalg.norm(b)
                     
        # The rotation axis (normalised).
        axis = np.cross(vector_orig, vector_fin)
        axis_len = np.linalg.norm(axis)
        if axis_len != 0.0:
            axis = axis / axis_len
    
        # Alias the axis coordinates.
        x = axis[0]
        y = axis[1]
        z = axis[2]
        
        # The rotation angle.
        angle = np.math.acos(np.dot(vector_orig, vector_fin))
    
        # Trig functions (only need to do this maths once!).
        ca = np.math.cos(angle)
        sa = np.math.sin(angle)
        R = np.identity(3)
        # Calculate the rotation matrix elements.
        R[0,0] = 1.0 + (1.0 - ca)*(x**2 - 1.0)
        R[0,1] = -z*sa + (1.0 - ca)*x*y
        R[0,2] = y*sa + (1.0 - ca)*x*z
        R[1,0] = z*sa+(1.0 - ca)*x*y
        R[1,1] = 1.0 + (1.0 - ca)*(y**2 - 1.0)
        R[1,2] = -x*sa+(1.0 - ca)*y*z
        R[2,0] = -y*sa+(1.0 - ca)*x*z
        R[2,1] = x*sa+(1.0 - ca)*y*z
        R[2,2] = 1.0 + (1.0 - ca)*(z**2 - 1.0)
        
        R = np.matrix(R)
        return R 
        
if __name__ == "__main__":
    j = ClutchControl( ClutchControl.MODE.simulation)
    
    