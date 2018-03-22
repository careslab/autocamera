from __common_imports__ import *
from mtm import mtm 

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
        
        
    def __init__(self, teleop_thread, mode = MODE.simulation):        
        self.__mode__ = mode
        self.teleop_thread = teleop_thread
        
        self.camera_clutch_pressed = False
        self.movement_scale = 1.5 
        self.joint_angles = [0,0,0,0]
        self.center = [0,0,0,0]
        
        self.flag = True
        
        self.mtml_pos = [0,0,0]
        self.mtml_joint_angles = [0,0,0,0,0,0,0]
        
        self.mtmr_pos = [0,0,0]
        self.mtmr_joint_angles = [0,0,0,0,0,0,0]
        
        self.mtmr_starting_point =  [0,0,0,0,0,0,0]
        
    def __init_nodes__(self):
#         rospy.init_node('ecm_clutch_control')
        
        self.hw_ecm = robot("ECM")
        self.__hw_mtmr__ = robot("MTMR")
        self.__hw_mtml__ = robot("MTML")
        
        self.pub_mtml_hw = rospy.Publisher('/dvrk/MTML/set_position_joint', JointState, queue_size=1)
        self.pub_mtmr_hw = rospy.Publisher('/dvrk/MTMR/set_position_joint', JointState, queue_size=1)
        
        self.pub_ecm_sim = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=10)
        self.__ecm_robot__ = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        self.__mtmr_robot__ = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
        self.__mtml_robot__ = URDF.from_parameter_server('/dvrk_mtml/robot_description')
        self.__psm1_robot__ = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        self.__psm2_robot__ = URDF.from_parameter_server('/dvrk_psm2/robot_description')
        
        self.__ecm_kin__ = KDLKinematics(self.__ecm_robot__, self.__ecm_robot__.links[0].name, self.__ecm_robot__.links[-1].name)
        self.__mtmr_kin__ = KDLKinematics(self.__mtmr_robot__, self.__mtmr_robot__.links[0].name, self.__mtmr_robot__.links[-1].name)
        self.__mtml_kin__ = KDLKinematics(self.__mtml_robot__, self.__mtml_robot__.links[0].name, self.__mtml_robot__.links[-1].name)
        self.__psm1_kin__ = KDLKinematics(self.__psm1_robot__, self.__psm1_robot__.links[0].name, self.__psm1_robot__.links[-1].name)
        self.__psm2_kin__ = KDLKinematics(self.__psm2_robot__, self.__psm2_robot__.links[0].name, self.__psm2_robot__.links[-1].name)
        
        self.hw_mtml_orientation = mtm('MTML')
        self.hw_mtmr_orientation = mtm('MTMR')
        
        # MTML lock orientation
#         self.__pub_lock_mtml_orientation__ = rospy.Publisher('/dvrk/MTML/lock_orientation', Quaternion, latch=True, queue_size = 1)
#         self.__pub_lock_mtmr_orientation__ = rospy.Publisher('/dvrk/MTMR/lock_orientation', Quaternion, latch=True, queue_size = 1)
        
        
        self.sub_ecm_cb = None
        if self.__mode__ == self.MODE.simulation:
            self.sub_ecm_cb = rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.__ecm_cb__)
        elif self.__mode__ == self.MODE.hardware:
            self.sub_ecm_cb = rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.__ecm_cb__)
#             self.hw_ecm.home()
            self.hw_ecm.move_joint_list([0.0,0.0,0.0,0.0], interpolate=True)
        
        self.camera_clutch_pressed = False
        self.head_sensor_pressed = False
        self.sub_camera_clutch_cb = rospy.Subscriber('/dvrk/footpedals/camera', Joy, self.camera_clutch_cb )
        self.__sub_headsensor__ = rospy.Subscriber('/dvrk/footpedals/coag', Joy, self.__headsensor_cb__ )
        self.mtml_starting_point = None
        
        self.sub_mtml_cart_cb = rospy.Subscriber('/dvrk/MTML/position_cartesian_local_current', PoseStamped, self.__mtml_cb__)
        self.sub_mtml_joint_cb = rospy.Subscriber('/dvrk/MTML/state_joint_current', JointState, self.mtml_joint_angles_cb)
        
        self.sub_mtmr_cart_cb = rospy.Subscriber('/dvrk/MTMR/position_cartesian_local_current', PoseStamped, self.__mtmr_cb__)
        self.sub_mtmr_joint_cb = rospy.Subscriber('/dvrk/MTMR/state_joint_current', JointState, self.mtmr_joint_angles_cb)
        
        self.sub_psm1_joint_cb = rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, self.psm1_joint_angles_cb)
        self.sub_psm2_joint_cb = rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, self.psm2_joint_angles_cb)
        
        self.T_mtml_pos_init = self.__mtml_kin__.forward([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.T_mtmr_pos_init = self.T_mtml_pos_init
#         self.hw_mtml_orientation.lock_orientation_as_is()
#         self.hw_mtml_orientation.unlock_orientation()
    
    def shutdown(self):
        try:
            self.sub_ecm_cb.unregister()
            self.sub_camera_clutch_cb.unregister()
            self.__sub_headsensor__.unregister()
            self.sub_mtml_cart_cb.unregister()
            self.sub_mtml_joint_cb.unregister()
            self.sub_mtmr_cart_cb.unregister()
            self.sub_mtmr_joint_cb.unregister()
            self.sub_psm1_joint_cb.unregister()
            self.sub_psm2_joint_cb.unregister()
            
            self.__hw_mtml__.unregister()
            self.__hw_mtmr__.unregister()
            self.hw_mtml_orientation.unregister()
            self.hw_mtmr_orientation.unregister()
            
            self.pub_mtml_hw.unregister()
            self.pub_mtmr_hw.unregister()
            self.pub_ecm_sim.unregister()
#             self.__pub_lock_mtml_orientation__.unregister()
#             self.__pub_lock_mtmr_orientation__.unregister()
            
            print( "Shutting down " + self.__class__.__name__)
            
        except(e):
            print("couldn't unregister all the topics")
            pass
#         rospy.signal_shutdown('shutting down ClutchControl')
        
    def set_scale(self, scale):
        self.movement_scale = scale
            
    def set_mode(self, mode):
        """ Values:
            MODE.simulation
            MODE.hardware
        """
        self.__mode__ = mode
        
            
    def spin(self):
        self.__init_nodes__()
        rospy.spin()

    def mtml_joint_angles_cb(self, msg):
        self.mtml_joint_angles = list(msg.position)
        
        T = self.__mtml_kin__.forward(self.mtml_joint_angles)
        pos = T[0:3,3]
        if self.camera_clutch_pressed:
            if type(self.mtml_starting_point) == NoneType:
                self.mtml_starting_point = pos
            
            # we may multiply the current_position and mtml_pos_before_clutch by some transformation matrix so
            # the hand controllers feel more intuitive
            
            movement_vector = pos-self.mtml_pos_before_clutch
            self.move_mtm_centerpoints()
#             print("movement_vector = {}, {}, {}".format(movement_vector[0], movement_vector[1], movement_vector[2]))
            self.mtml_pos = pos
            self.ecm_pan_tilt(movement_vector)
        else:
#             self.center = self.joint_angles
            try:
                self.mtml_pos_before_clutch = pos
            except:
                pass
            
    
    def mtmr_joint_angles_cb(self, msg):
        self.mtmr_joint_angles = list(msg.position)
        
        T = self.__mtmr_kin__.forward(self.mtmr_joint_angles)
        
        pos = T[0:3,3]
        if self.camera_clutch_pressed:
            if type(self.mtmr_starting_point) == NoneType:
                self.mtmr_starting_point = pos
            
            movement_vector = pos-self.mtmr_pos_before_clutch
            self.mtmr_pos = pos
        else:
            try:
                self.mtmr_pos_before_clutch = pos
            except:
                pass
    
    def psm1_joint_angles_cb(self,msg):
        self.psm1_joint_angles = list(msg.position)
    
    def psm2_joint_angles_cb(self,msg):
        self.psm2_joint_angles = list(msg.position)
    
    def __mtml_cb__(self, msg):
        return
#         msg.pose.position.x
        if self.camera_clutch_pressed:
            if type(self.mtml_starting_point) == NoneType:
                self.mtml_starting_point = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            
            # we may multiply the current_position and mtml_pos_before_clutch by some transformation matrix so
            # the hand controllers feel more intuitive
            
            current_position = self.__mtml_kin__.forward(list(self.mtml_joint_angles)[0:-1])[0:3,3] 
            movement_vector = current_position-self.mtml_pos_before_clutch
            self.move_mtm_centerpoints()
#             print("movement_vector = {}, {}, {}".format(movement_vector[0], movement_vector[1], movement_vector[2]))
            self.mtml_pos = current_position
        else:
#             self.center = self.joint_angles
            try:
                self.mtml_pos_before_clutch = self.__mtml_kin__.forward(list(self.mtml_joint_angles)[0:-1])[0:3,3]
            except:
                pass
    
    def __mtmr_cb__(self, msg):
        return
        if self.camera_clutch_pressed:
            pass
            if type(self.mtmr_starting_point) == NoneType:
                self.mtmr_starting_point = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            current_position = self.__mtmr_kin__.forward(list(self.mtmr_joint_angles)[0:-1])[0:3,3] 
            movement_vector = current_position-self.mtmr_starting_point
            self.mtmr_pos = current_position
#             self.ecm_pan_tilt(movement_vector[0:2])
        else:
#             self.center = self.joint_angles
            try :
                self.mtmr_pos_before_clutch = self.__mtmr_kin__.forward(list(self.mtmr_joint_angles)[0:-1])[0:3,3]
            except:
                pass
    
    # To be completed
    def move_mtm_centerpoints(self):
        left = self.mtml_pos_before_clutch
        right = self.mtmr_pos_before_clutch
        
        mtml_pos = self.__mtml_kin__.forward(self.mtml_joint_angles)[0:3,3]
        mtmr_pos = self.__mtmr_kin__.forward(self.mtmr_joint_angles)[0:3,3]
        
        mid = (left+right)/2.0
        
        if type(mtml_pos) != NoneType:
            ml_vector = mid-left
            mr_vector = right-mid
            
            new_mid = mtml_pos + ml_vector
            new_mtmr_position = new_mid + mr_vector
            
            mtmr_pose = self.__mtmr_kin__.forward(self.mtmr_joint_angles)
            mtmr_pose[0:3, 3] = new_mtmr_position.reshape(3,1)
            mtmr_joint_angles = self.__mtmr_kin__.inverse(mtmr_pose)
#             print('mtmr_joint_angles = ' + mtmr_joint_angles.__str__())
            
            mtmr_joint_angles = [float(i) for i in mtmr_joint_angles]
            mtmr_joint_angles.append(0.0)
            self.__hw_mtmr__.move_joint_list(mtmr_joint_angles[0:3], [0,1,2], interpolate=False)
            
#             self.move_mtm_out_of_the_way()
             
        
    def enable_teleop(self):
        self.__hw_mtml__.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
        self.__hw_mtml__.set_wrench_body_force([0,0,0])
        self.__hw_mtml__.set_gravity_compensation(True)
        
        self.__hw_mtmr__.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
        self.__hw_mtmr__.set_wrench_body_force([0,0,0])
        self.__hw_mtmr__.set_gravity_compensation(True)
        
    def disable_teleop(self):
        self.__hw_mtml__.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
        self.__hw_mtml__.set_gravity_compensation(False)
        
        self.__hw_mtmr__.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
        self.__hw_mtmr__.set_gravity_compensation(False)
                
    def __headsensor_cb__(self, msg):
        if msg.buttons[0] == 1:
            self.head_sensor_pressed = True
#             self.enable_teleop()
        else:
            self.head_sensor_pressed = False
#             self.__hw_mtml__.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
#             self.__hw_mtmr__.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
                        
    def camera_clutch_cb(self, msg):
        if msg.buttons[0] == 1 and self.head_sensor_pressed:
            if self.camera_clutch_pressed == False:
                self.camera_clutch_pressed = True
                self.mtml_starting_point = None
                self.teleop_thread.pause()
                self.hw_mtml_orientation.lock_orientation_as_is()
#                 self.hw_mtmr_orientation.lock_orientation_as_is()
#                 self.teleop_thread.lock_mtm_orientations()
            
        elif self.camera_clutch_pressed == True:
            self.camera_clutch_pressed = False
            if self.head_sensor_pressed:
                self.teleop_thread.resume()
                self.hw_mtml_orientation.unlock_orientation()
#                 self.hw_mtmr_orientation.unlock_orientation()
            
            
                            
    def __ecm_cb__(self, msg):
        if self.__mode__ == self.MODE.simulation:
            self.joint_angles = msg.position[0:2] + msg.position[-2:]
        elif self.__mode__ == self.MODE.hardware:
            self.joint_angles = msg.position[0:3] + tuple([0])
            
        if self.camera_clutch_pressed == False:
            if self.__mode__ == self.MODE.simulation:
                self.center = msg.position[0:2] + msg.position[-2:]
            elif self.__mode__ == self.MODE.hardware:
                self.center = msg.position[0:3] + tuple([0])
            self.center_cart = np.array(self.__ecm_kin__.FK(self.center)[0])
#         print("self.center is : " + self.center.__str__())  
#         print("ecm joint angles are : " + msg.position.__str__())
#         print("self.joint_angles is : " + self.joint_angles.__str__())
    
    def ecm_pan_tilt(self, movement_vector):
        q = []
        q = self.joint_angles
        movement_vector = [float(i) for i in movement_vector]
        if q:
            q = list(q)
            q[0] = self.center[0] + movement_vector[0] * self.movement_scale # pitch
            q[1] = self.center[1] - movement_vector[1] * self.movement_scale # yaw
            q[2] = self.center[2] + movement_vector[2] *.2 * self.movement_scale # insertion
            
            q = [round(i,4) for i in q]
#             q = [i+j for i,j in zip(self.center, q)]
            self.move_ecm(q)
    
    # move ecm based on joint angles either in simulation or hardware
    def move_ecm(self, joint_angles):
        if self.__mode__ == self.MODE.simulation:
            msg = JointState()
            msg.position = joint_angles
            msg.name = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
            self.pub_ecm_sim.publish(msg)            
        elif self.__mode__ == self.MODE.hardware:
            self.hw_ecm.move_joint_list(joint_angles, interpolate=False)
    
    def ecm_inverse(self, goal_pos):
        key_hole,_ = self.__ecm_kin__.FK([0,0,0,0])
        safe_angles = list(self.joint_angles)
        safe_angles[2] = 0.14
        
        ecm_pose = self.__ecm_kin__.forward(safe_angles)
        ab_vector = (goal_pos - key_hole)
        
        b, _ = self.__ecm_kin__.FK(safe_angles)
        
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
            p = self.__ecm_kin__.inverse(ecm_pose)
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
