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
from geometry_msgs.msg import PoseStamped, 
import std_msgs
from std_msgs.msg import Bool
from std_msgs.msg import String

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
        self.camera_clutch_pressed = False
        self.movement_scale = 1 
        self.joint_angles = [0,0,0,0]
        self.center = [0,0,0,0]
        
        self.flag = True
        
        self.mtml_pos = [0,0,0]
        self.mtml_joint_angles = [0,0,0,0,0,0,0]
        
        self.mtmr_pos = [0,0,0]
        self.mtmr_joint_angles = [0,0,0,0,0,0,0]
        
        self.mtmr_starting_point =  [0,0,0,0,0,0,0]
        
        self.__init_nodes__()
        self.spin()
        
    def __init_nodes__(self):
        rospy.init_node('ecm_clutch_control')
        
        self.ecm_hw = robot("ECM")
        self.mtmr_hw = robot("MTMR")
        self.mtml_hw = robot("MTML")
        
        self.mtml_hw_pub = rospy.Publisher('/dvrk/MTML/set_position_joint', JointState, queue_size=1)
        self.mtmr_hw_pub = rospy.Publisher('/dvrk/MTMR/set_position_joint', JointState, queue_size=1)
        
        self.ecm_sim = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=10)
        self.ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        self.mtmr_robot = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
        self.mtml_robot = URDF.from_parameter_server('/dvrk_mtml/robot_description')
        self.psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        self.psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
        
        self.ecm_kin = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[-1].name)
        self.mtmr_kin = KDLKinematics(self.mtmr_robot, self.mtmr_robot.links[0].name, self.mtmr_robot.links[-1].name)
        self.mtml_kin = KDLKinematics(self.mtml_robot, self.mtml_robot.links[0].name, self.mtml_robot.links[-1].name)
        self.psm1_kin = KDLKinematics(self.psm1_robot, self.psm1_robot.links[0].name, self.psm1_robot.links[-1].name)
        self.psm2_kin = KDLKinematics(self.psm2_robot, self.psm2_robot.links[0].name, self.psm2_robot.links[-1].name)
        
        self.mtml_orientation = mtm('MTML')
        self.mtmr_orientation = mtm('MTMR')
        
        self.mtml_psm2_orientation = rospy.Publisher('/dvrk/MTML_PSM2/lock_rotation', Bool, queue_size=1, latch=True )
        self.mtml_psm2_translation = rospy.Publisher('/dvrk/MTML_PSM2/lock_translation', Bool, queue_size=1, latch=True )
        
        self.mtmr_psm1_orientation = rospy.Publisher('/dvrk/MTMR_PSM1/lock_rotation', Bool, queue_size=1, latch=True )
        self.mtmr_psm1_translation = rospy.Publisher('/dvrk/MTMR_PSM1/lock_translation', Bool, queue_size=1, latch=True )
        
        self.mtmr_psm1_teleop = rospy.Publisher('/dvrk/MTMR_PSM1/set_desired_state', String, latch=True, queue_size=1)
        self.mtml_psm2_teleop = rospy.Publisher('/dvrk/MTML_PSM2/set_desired_state', String, latch=True, queue_size=1)
        
        self.sub_ecm_cb = None
        if self.__mode__ == self.MODE.simulation:
            self.sub_ecm_cb = rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.ecm_cb)
        elif self.__mode__ == self.MODE.hardware:
            self.sub_ecm_cb = rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.ecm_cb)
#             self.ecm_hw.home()
            self.ecm_hw.move_joint_list([0.0,0.0,0.07,0.0], interpolate=True)
        
        self.camera_clutch_pressed = False
        self.head_sensor_pressed = False
        self.sub_camera_clutch_cb = rospy.Subscriber('/dvrk/footpedals/camera_minus', Joy, self.camera_clutch_cb )
        self.sub_headsensor_cb = rospy.Subscriber('/dvrk/footpedals/coag', Joy, self.camera_headsensor_cb )
        self.mtml_starting_point = None
        
        self.sub_mtml_cart_cb = rospy.Subscriber('/dvrk/MTML/position_cartesian_local_current', PoseStamped, self.mtml_cb)
        self.sub_mtml_joint_cb = rospy.Subscriber('/dvrk/MTML/state_joint_current', JointState, self.mtml_joint_angles_cb)
        
        self.sub_mtmr_cart_cb = rospy.Subscriber('/dvrk/MTMR/position_cartesian_local_current', PoseStamped, self.mtmr_cb)
        self.sub_mtmr_joint_cb = rospy.Subscriber('/dvrk/MTMR/state_joint_current', JointState, self.mtmr_joint_angles_cb)
        
        self.sub_psm1_joint_cb = rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, self.psm1_joint_angles_cb)
        self.sub_psm2_joint_cb = rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, self.psm2_joint_angles_cb)
        
#         self.mtml_orientation.lock_orientation_as_is()
#         self.mtml_orientation.unlock_orientation()
    
    def shutdown(self):
        try:
            self.sub_ecm_cb.unregister()
            self.sub_camera_clutch_cb.unregister()
            self.sub_headsensor_cb.unregister()
            self.sub_mtml_cart_cb.unregister()
            self.sub_mtml_joint_cb.unregister()
            self.sub_mtmr_cart_cb.unregister()
            self.sub_mtmr_joint_cb.unregister()
            self.sub_psm1_joint_cb.unregister()
            self.sub_psm2_joint_cb.unregister()
            
        except(e):
            pass
        rospy.signal_shutdown('shutting down ClutchControl')
        
    def set_mode(self, mode):
        """ Values:
            MODE.simulation
            MODE.hardware
        """
        self.__mode__ = mode
            
    def spin(self):
        rospy.spin()

    def mtml_joint_angles_cb(self, msg):
        self.mtml_joint_angles = list(msg.position)
        self.move_mtm_out_of_the_way()
    
    def mtmr_joint_angles_cb(self, msg):
        self.mtmr_joint_angles = list(msg.position)
        
    def psm1_joint_angles_cb(self,msg):
        self.psm1_joint_angles = list(msg.position)
    
    def psm2_joint_angles_cb(self,msg):
        self.psm2_joint_angles = list(msg.position)
    
    def mtml_cb(self, msg):
#         msg.pose.position.x
        if self.camera_clutch_pressed:
            if type(self.mtml_starting_point) == NoneType:
                self.mtml_starting_point = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            
            # we may multiply the current_position and mtml_pos_before_clutch by some transformation matrix so
            # the hand controllers feel more intuitive
            
            current_position = self.mtml_kin.forward(list(self.mtml_joint_angles)[0:-1])[0:3,3] 
            movement_vector = current_position-self.mtml_pos_before_clutch
            self.move_mtm_centerpoints()
            print("movement_vector = {}, {}, {}".format(movement_vector[0], movement_vector[1], movement_vector[2]))
            self.mtml_pos = current_position
            self.ecm_pan_tilt(movement_vector)
        else:
#             self.center = self.joint_angles
            try:
                self.mtml_pos_before_clutch = self.mtml_kin.forward(list(self.mtml_joint_angles)[0:-1])[0:3,3]
            except:
                pass
    
    def mtmr_cb(self, msg):
        pass
        if self.camera_clutch_pressed:
            pass
            if type(self.mtmr_starting_point) == NoneType:
                self.mtmr_starting_point = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            current_position = self.mtmr_kin.forward(list(self.mtmr_joint_angles)[0:-1])[0:3,3] 
            movement_vector = current_position-self.mtmr_starting_point
            self.mtmr_pos = current_position
#             self.ecm_pan_tilt(movement_vector[0:2])
        else:
#             self.center = self.joint_angles
            try :
                self.mtmr_pos_before_clutch = self.mtmr_kin.forward(list(self.mtmr_joint_angles)[0:-1])[0:3,3]
            except:
                pass
    
    def move_mtm_out_of_the_way(self):
        if self.head_sensor_pressed == True and self.camera_clutch_pressed == False:
            if self.mtml_hw.get_robot_state() == 'DVRK_POSITION_GOAL_CARTESIAN' and self.flag == True:
                self.flag == False
                ml = self.mtml_joint_angles[3]
                mr = self.mtml_joint_angles[3]
                delta = .1
                
                mtml_joints = JointState()
                mtml_joints.name = ['outer_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_platform', 'wrist_pitch', 'wrist_yaw', 'wrist_roll', 'finger_grips']
                mtml_joints.position = self.mtml_joint_angles
                mtml_joints.position[3] += (-1.58 -mtml_joints.position[3])/10.0  
                mtml_joints.effort = [0, 0, 0, delta, 0, 0, 0, 0]
                print("mtml_joints = " + mtml_joints.__str__())
                self.mtml_hw_pub.publish(mtml_joints)
                
                mtmr_joints = JointState()
                mtmr_joints.name = ['outer_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_platform', 'wrist_pitch', 'wrist_yaw', 'wrist_roll', 'finger_grips']
                mtmr_joints.position = self.mtmr_joint_angles
                new_p = list(mtmr_joints.position)
                new_p[3] = 1.58
                mtmr_joints.position[3] +=  (1.58 - mtmr_joints.position[3])/10.0 
                mtmr_joints.effort = [0, 0, 0, delta, 0, 0, 0, 0]
                self.mtmr_hw_pub.publish(mtmr_joints)
                
    # To be completed
    def move_mtm_centerpoints(self):
        left = self.mtml_pos_before_clutch
        right = self.mtmr_pos_before_clutch
        
        mtml_pos = self.mtml_kin.forward(list(self.mtml_joint_angles)[0:-1])[0:3,3]
        mtmr_pos = self.mtmr_kin.forward(list(self.mtmr_joint_angles)[0:-1])[0:3,3]
        
        mid = (left+right)/2.0
        
        if type(mtml_pos) != NoneType:
            ml_vector = mid-left
            mr_vector = right-mid
            
            new_mid = mtml_pos + ml_vector
            new_mtmr_position = new_mid + mr_vector
            
            mtmr_pose = self.mtmr_kin.forward(list(self.mtmr_joint_angles)[0:-1])
            mtmr_pose[0:3, 3] = new_mtmr_position.reshape(3,1)
            mtmr_joint_angles = self.mtmr_kin.inverse(mtmr_pose)
#             print('mtmr_joint_angles = ' + mtmr_joint_angles.__str__())
            
            mtmr_joint_angles = [float(i) for i in mtmr_joint_angles]
            mtmr_joint_angles.append(0.0)
            self.mtmr_hw.move_joint_list(mtmr_joint_angles[0:3], [0,1,2], interpolate=False)
            
#             self.move_mtm_out_of_the_way()
             
        
        
    def enable_teleop(self):
        self.mtmr_psm1_teleop.publish(String("ENABLED"))
        self.mtml_psm2_teleop.publish(String("ENABLED"))
        
        
    def disable_teleop(self):
        self.mtml_orientation.lock_orientation_as_is()
        self.mtmr_orientation.lock_orientation_as_is()
        
        self.mtmr_psm1_teleop.publish(String("DISABLED"))
        self.mtml_psm2_teleop.publish(String("DISABLED"))
                
    def camera_headsensor_cb(self, msg):
        if msg.buttons[0] == 1:
            self.head_sensor_pressed = True
            self.enable_teleop()
        else:
            self.head_sensor_pressed = False
            self.mtml_hw.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
            self.mtmr_hw.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
                        
    def camera_clutch_cb(self, msg):
        if msg.buttons[0] == 1 and self.head_sensor_pressed: # Camera clutch pressed
            self.camera_clutch_pressed = True
            self.mtml_starting_point = None
            self.disable_teleop()
            
        else: # Camera clutch not pressed anymore
            self.camera_clutch_pressed = False
#             self.center = self.joint_angles
            if self.head_sensor_pressed:
                self.enable_teleop()
                            
    def ecm_cb(self, msg):
        if self.__mode__ == self.MODE.simulation:
            self.joint_angles = msg.position[0:2] + msg.position[-2:]
        elif self.__mode__ == self.MODE.hardware:
            self.joint_angles = msg.position[0:3] + tuple([0])
            
        if self.camera_clutch_pressed == False:
            if self.__mode__ == self.MODE.simulation:
                self.center = msg.position[0:2] + msg.position[-2:]
            elif self.__mode__ == self.MODE.hardware:
                self.center = msg.position[0:3] + tuple([0])
            self.center_cart = np.array(self.ecm_kin.FK(self.center)[0])
            
    
    def ecm_pan_tilt(self, movement_vector):
        q = []
        q = self.joint_angles
        print("ecm joint angles are : " + q.__str__())
        movement_vector = [float(i) for i in movement_vector]
        if q:
#             ee = np.array(self.ecm_kin.FK(q)[0])
#             ee[0] = self.center_cart[0] + movement_vector[0] * .01
#             ee[1] = self.center_cart[1] + movement_vector[1] * .01
#             ee[2] = self.center_cart[2] + movement_vector[2] * .01
#               
#             ee_inv = self.ecm_inverse(ee)
#             if type(ee_inv) == NoneType:
#                 return
#               
#             new_joint_angles = [float(i) for i in ee_inv]
#             new_joint_angles[2] = q[2]
#             self.move_ecm(new_joint_angles)
             
            q = list(q)
            q[0] = self.center[0] + movement_vector[0] * self.movement_scale # pitch
            q[1] = self.center[1] - movement_vector[1] * self.movement_scale # yaw
            q[2] = self.center[2] - movement_vector[2] *.2 * self.movement_scale # insertion
            
            q = [round(i,4) for i in q]
#             q = [i+j for i,j in zip(self.center, q)]
            print(q)
            self.move_ecm(q)
    
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
        safe_angles[2] = 0.14
        
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
            rospy.logerr('error, cannot do inverse kinematics of ecm')
        if type(p) != NoneType: 
            p[3] = 0
#             p[2] -= 0.14
            new_joint_angles = p
        else:
            from colorama import Fore, Back
            print (Fore.GREEN + Back.YELLOW+ "inverse Kinematics is failing") 
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
    j = ClutchControl( ClutchControl.MODE.hardware)
    
    