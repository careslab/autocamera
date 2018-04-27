from __common_imports__ import *
from autocamera_algorithm import Autocamera
from rospy.core import rospydebug
class ClutchlessSystem:
    """!
        The ClutchlessSystem class intends to automate the movements 
        of the camera and MTM repositioning.
        
        TO DO:
            - Create 3D deadzone for autocamera
                - Display it in RViz [Done]
                - Compute the correct parameters for the deadzone
            - Have the camera move when a tool hits the edge of the deadzone
                - Detect when a tool is hitting the edge
                - Determine which tool
                - Move   the camera with the tool as long as it is touching the edge
            - Figure out a way to use clutchless system with zooming
            - Create a fitness function for MTM and PSM relation in the camera view
            - Implement clutchless system when the tools are in camera view
            - Implement the clutchless sytem when the camera is moving
    """
    
    class MODE:
        """
            simulation mode: Use the hardware for the master side, 
                            and simulation for the ECM
            hardware mode: Use hardware for both the master side,
                            and the ECM
        """
        simulation = "SIMULATION"
        hardware = "HARDWARE"
        
    # @param mode hardware or simulation mode
    #  @param self The object pointer.
    def __init__(self, mode = MODE.simulation):
        """!
        Initialize the parameters for the Teleop_class
        @param mode : hardwre or simulation mode
        """
        
        self.__mode__ = mode
        self.__autocamera__ = Autocamera()
        self.__cam_info__ = {'left':CameraInfo(), 'right':CameraInfo()}
        
        ## The scale of movements from MTMs to PSMs
        self.scale = 0.3
        self.__x_scale__ = self.scale
        self.__y_scale__ = self.scale
        self.__z_scale__ = self.scale
        
        self.__mtml_translations__ = None
        # Hand controller's desired position
        self.__mtml_home_position__ = None
        self.__mtmr_home_position__ = None
        self.__mtml_dir__ = [0,0,0] # mtml movement direction
        self.__mtmr_dir__ = [0,0,0]
        
        self.__enabled__ = False
        self.__clutch_active__ = False
         
        self.__mtml_last_pos__ = None
        self.__mtml_last_rot__ = None
        self.__mtmr_last_pos__ = None
        self.__mtmr_last_rot__ = None
    
        self.__last_good_psm1_transform__ = None
        self.__last_good_psm2_transform__ = None
        
        self.__mtml_gripper__ = None
        self.__mtmr_gripper__ = None
        
        self.__psm1_last_jnt__ = None
        self.__psm2_last_jnt__ = None
        
        self.__ecm_last_jnt__ = None
        
        self.__mtml_first_pos__ = None
        self.__mtmr_first_pos__ = None
        self.__psm1_first_pos__ = None
        self.__psm2_first_pos__ = None
        
        self.__T_ecm__ = None
        self.__T_mtml_000__ = None
        self.__T_mtmr_000__ = None
        self.__arms_homed__ = False
        self.__paused__ = False
        self.__reenable_teleop__ = False
        
        
        self.__mtml_joint_names__ = ['outer_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_platform', 'wrist_pitch', 'wrist_yaw', 'wrist_roll']
        self.__mtmr_joint_names__ = ['outer_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_platform', 'wrist_pitch', 'wrist_yaw', 'wrist_roll']
        self.__psm1_joint_names__ =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
        self.__psm2_joint_names__ =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
        self.__ecm_joint_names__ = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
                
        
    def __init__nodes(self):
        """!
        Initialize the ros publishers and subscribers
        """
        
        self.__deadzone_pub__ = rospy.Publisher('/deadzone', PolygonStamped)
        
        self.__mtml_robot__ = URDF.from_parameter_server('/dvrk_mtml/robot_description')
        self.__mtmr_robot__ = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
        self.__psm2_robot__ = URDF.from_parameter_server('/dvrk_psm2/robot_description')
        self.__psm1_robot__ = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        self.__ecm_robot__ = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        
        self.__mtml_kin__ = KDLKinematics(self.__mtml_robot__, self.__mtml_robot__.links[0].name, self.__mtml_robot__.links[-1].name)
        self.__mtmr_kin__ = KDLKinematics(self.__mtmr_robot__, self.__mtmr_robot__.links[0].name, self.__mtmr_robot__.links[-1].name)
        self.__psm1_kin__ = KDLKinematics(self.__psm1_robot__, self.__psm1_robot__.links[0].name, self.__psm1_robot__.links[-1].name)
        self.__psm2_kin__ = KDLKinematics(self.__psm2_robot__, self.__psm2_robot__.links[0].name, self.__psm2_robot__.links[-1].name)
        self.__ecm_kin__ = KDLKinematics(self.__ecm_robot__, self.__ecm_robot__.links[0].name, self.__ecm_robot__.links[-1].name)
        
        # Publish to PSMs simulation
        self.__pub_psm1__ = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=1)
        self.__pub_psm2__ = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=1)
        
        # Publish to MTMs simulation
        self.__pub_mtml__ = rospy.Publisher('/dvrk_mtml/joint_states_robot', JointState, queue_size=10)
        self.__pub_mtmr__ = rospy.Publisher('/dvrk_mtmr/joint_states_robot', JointState, queue_size=10)
    
        # Subscribe to MTMs
        self.__sub_mtml__ = None; self.__sub_mtmr__ = None
        if self.__mode__ == self.MODE.simulation:
            self.__sub_mtml__ = rospy.Subscriber('/dvrk_mtml/joint_states', JointState, self.__mtml_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_mtmr__ = rospy.Subscriber('/dvrk_mtmr/joint_states', JointState, self.__mtmr_cb__, queue_size=1, tcp_nodelay=True)
        elif self.__mode__ == self.MODE.hardware:
            self.__sub_mtml__ = rospy.Subscriber('/dvrk/MTML/state_joint_current', JointState, self.__mtml_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_mtmr__ = rospy.Subscriber('/dvrk/MTMR/state_joint_current', JointState, self.__mtmr_cb__, queue_size=1, tcp_nodelay=True)
        
        # subscribe to head sensor
        self.__sub_headsensor__ = rospy.Subscriber('/dvrk/footpedals/coag', Joy, self.__headsensor_cb__ , queue_size=1, tcp_nodelay=True)
        
        # subscribe to camera info
        self.__sub_caminfo__ = rospy.Subscriber('/fakecam_node/camera_info', CameraInfo, self.__get_cam_info, queue_size=1, tcp_nodelay=True)
        
        # Subscribe to PSMs
        self.__sub_psm1__ = None; self.__sub_psm2__ = None
        if self.__mode__ == self.MODE.simulation:
            self.__sub_psm1__ = rospy.Subscriber('/dvrk_psm1/joint_states', JointState, self.__psm1_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_psm2__ = rospy.Subscriber('/dvrk_psm2/joint_states', JointState, self.__psm2_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_ecm__ = rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.__ecm_cb__, queue_size=1, tcp_nodelay=True)
        else:
            self.__sub_psm1__ = rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, self.__psm1_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_psm2__ = rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, self.__psm2_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_ecm__ = rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.__ecm_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_mtml_gripper__ = rospy.Subscriber('/dvrk/MTML/gripper_position_current', Float32, self.__mtml_gripper_cb__, queue_size=1, tcp_nodelay=True)
            self.__sub_mtmr_gripper__ = rospy.Subscriber('/dvrk/MTMR/gripper_position_current', Float32, self.__mtmr_gripper_cb__, queue_size=1, tcp_nodelay=True)
            
        # MTM repositioning clutch
        self.__sub_clutch__ = rospy.Subscriber('/dvrk/footpedals/clutch', Joy, self.__clutch_cb__, queue_size=1, tcp_nodelay=True)
            
        
        # Publish to ECM simulation
        self.__pub_ecm__ = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=10)
        
        if self.__mode__ == self.MODE.hardware:
            # Translation and orientation lock
            self.__lock_mtml_psm2_orientation__ = rospy.Publisher('/dvrk/MTML_PSM2/lock_rotation', Bool, queue_size=1, latch=True )
            self.__lock_mtml_psm2_translation__ = rospy.Publisher('/dvrk/MTML_PSM2/lock_translation', Bool, queue_size=1, latch=True )
            
            self.__lock_mtmr_psm1_orientation__ = rospy.Publisher('/dvrk/MTMR_PSM1/lock_rotation', Bool, queue_size=1, latch=True )
            self.__lock_mtmr_psm1_translation__ = rospy.Publisher('/dvrk/MTMR_PSM1/lock_translation', Bool, queue_size=1, latch=True )
    
            # MTML lock orientation
            self.__pub_lock_mtml_orientation__ = rospy.Publisher('/dvrk/MTML/lock_orientation', Quaternion, latch=True, queue_size = 1)
            self.__pub_lock_mtmr_orientation__ = rospy.Publisher('/dvrk/MTMR/lock_orientation', Quaternion, latch=True, queue_size = 1)
            
            self.__pub_unlock_mtml_orientation__ = rospy.Publisher('/dvrk/MTML/unlock_orientation', Empty, latch=True, queue_size = 1)
            self.__pub_unlock_mtmr_orientation__ = rospy.Publisher('/dvrk/MTMR/unlock_orientation', Empty, latch=True, queue_size = 1)
            
            self.__pub_mtmr_psm1_teleop__ = rospy.Publisher('/dvrk/MTMR_PSM1/set_desired_state', String, latch=True, queue_size=1)
            self.__pub_mtml_psm2_teleop__ = rospy.Publisher('/dvrk/MTML_PSM2/set_desired_state', String, latch=True, queue_size=1)
            
            # Wrist Adjustments
            self.__mtml_wrist_adjustment__ = rospy.Publisher('/dvrk/MTML/run_wrist_adjustment', Empty, latch=True, queue_size=1)
            self.__mtmr_wrist_adjustment__ = rospy.Publisher('/dvrk/MTMR/run_wrist_adjustment', Empty, latch=True, queue_size=1)
            
            # Access psm hardware
            self.__hw_psm1__ = robot('PSM1')
            self.__hw_psm2__ = robot('PSM2')
            
            # Access mtm hardware
            self.__hw_mtml__ = robot('MTML')
            self.__hw_mtmr__ = robot('MTMR')
        
            # Access ecm hardware
            self.__hw_ecm__ = robot('ECM')
        
        self.home_arms()
        
        if self.__mode__ == self.MODE.simulation:
            self.enable_teleop()

    
    def run_once(f):
        def wrapper(*args, **kwargs):
            if not wrapper.has_run:
                wrapper.has_run = True
                return_value = f(*args, **kwargs)
                wrapper.has_run = False
                return return_value
        wrapper.has_run = False
        return wrapper
    
    @run_once
    def move_arm_joints(self, arm_name, joints, inter = False):
        
        # Make sure the format is correct
        joints = [float(i) for i in joints]
        
        if self.__mode__ == self.MODE.simulation:
            msg = JointState()
            if arm_name.lower() == 'mtml':
                msg.name = self.__mtml_joint_names__
                msg.position = joints
                self.__pub_mtml__.publish(msg)
            elif arm_name.lower() == 'mtmr':
                msg.position = joints
                msg.name = self.__mtmr_joint_names__
                self.__pub_mtmr__.publish(msg)
            elif arm_name.lower() == 'psm1':
                msg.name =  self.__psm1_joint_names__
                msg.position = joints
                self.__pub_psm1__.publish(msg)
            elif arm_name.lower() == 'psm2':
                msg.name =  self.__psm2_joint_names__
                msg.position = joints
                self.__pub_psm2__.publish(msg)
            elif arm_name.lower() == 'ecm':
                msg.name = self.__ecm_joint_names__
                msg.position = joints
                self.__pub_ecm__.publish(msg)
        
        if self.__mode__ == self.MODE.hardware:
            if arm_name.lower() == 'mtml':
                return self.__hw_mtml__.move_joint_list(joints, interpolate=inter)
            elif arm_name.lower() == 'mtmr':
                return self.__hw_mtmr__.move_joint_list(joints, interpolate=inter)
            elif arm_name.lower() == 'psm1':
                return self.__hw_psm1__.move_joint_list(joints, interpolate=inter)
            elif arm_name.lower() == 'psm2':
                return self.__hw_psm2__.move_joint_list(joints, interpolate=inter)
            elif arm_name.lower() == 'ecm':
                return self.__hw_ecm__.move_joint_list(joints, interpolate=inter)
            
        return True
        
    def rehome(self):
        """!
        Home all the arms again
        """
        self.__arms_homed__ = False
        self.home_arms()
        
    def home_arms(self):
        """!
        Move the psms and mtms to a preferred initial position
        """
        
        if self.__arms_homed__ :
            return
        
        q_psm1 = [0.125, 0.237, 0.137, 0.839, -0.122, -0.148, -0.174]
        q_psm2 = [-0.015, -0.050, 0.149, -0.988, -0.183, -0.057, -0.174]
        
        q_mtml = [0.086, 0.008, 0.141, -1.498, 0.074, -0.156, 0.005]
        q_mtmr = [0.131, -0.061, 0.165, 1.529, 0.284, 0.142, 0.053]
        
        if self.__mode__ == self.MODE.hardware:
            q_mtml.append(0.0)
            q_mtmr.append(0.0)
        
        for i in range(5): 
            r_psm1 = self.move_arm_joints('psm1', q_psm1, inter=True)
            rospy.sleep(.1)
            r_psm2 = self.move_arm_joints('psm2', q_psm2, inter=True)
            rospy.sleep(.1)
            r_mtml = self.move_arm_joints('mtml',  q_mtml, inter=True)
            rospy.sleep(.1)
            r_mtmr = self.move_arm_joints('mtmr', q_mtmr, inter=True)
            rospy.sleep(.1)
            r_ecm = self.move_arm_joints('ecm', [0.0, 0.0, 0.0, 0.0], inter=True)
            rospy.sleep(.1)
            if self.__mode__ == self.MODE.hardware:
                break
        
        if r_psm1 * r_psm2 * r_mtml * r_mtmr:
            self.__arms_homed__ = True
                
    def shutdown(self):
        """!
        Unregister all the ros publishers and subscribers
        """
        self.shut_down()
        
    def shut_down(self):
        """!
        Same as shutdown
        """
        
        self.__deadzone_pub__.unregister()
        
        self.__sub_mtml__.unregister()
        self.__sub_mtmr__.unregister()
        self.__sub_psm1__.unregister()
        self.__sub_psm2__.unregister()
        self.__sub_ecm__.unregister()
        self.__sub_headsensor__.unregister()
        self.__sub_clutch__.unregister()
        self.__sub_caminfo__.unregister()
        
        self.__pub_psm1__.unregister()
        self.__pub_psm2__.unregister()
        self.__pub_mtml__.unregister()
        self.__pub_mtmr__.unregister()
        self.__pub_ecm__.unregister()
        
        if self.__mode__ == self.MODE.hardware:
            self.__pub_lock_mtml_orientation__.unregister()
            self.__pub_lock_mtmr_orientation__.unregister()
            self.__pub_unlock_mtml_orientation__.unregister()
            self.__pub_unlock_mtmr_orientation__.unregister()
            
            self.__hw_mtml__.unregister()
            self.__hw_mtmr__.unregister()
            self.__hw_psm1__.unregister()
            self.__hw_psm2__.unregister()
            self.__hw_ecm__.unregister()
        
            self.__sub_mtml_gripper__.unregister()
            self.__sub_mtmr_gripper__.unregister()
    
    def set_mode(self, mode):
        """!
        Set the operation mode to either simulation or hardware
        @param mode : MODE.simulation or MODE.hardware
        """
        self.__mode__ = mode
        
    def spin(self):
        self.__init__nodes()
        rospy.spin()
        
    def pause(self):
        """!
        Pause the teleoperation
        """
        self.__enabled__ = False
        
    def resume(self):
        """!
        Resume the teleopration
        """
        self.__enabled__ = True
                
    def enable_teleop(self):
        """!
        Enable teleoperation
        """
        self.__enabled__ = True
        
        self.__mtml_first_pos__ = self.__mtml_last_pos__
        self.__mtmr_first_pos__ = self.__mtmr_last_pos__
        self.__psm1_first_pos__, _ = self.__psm1_kin__.FK( self.__psm1_last_jnt__)
        self.__psm2_first_pos__, _ = self.__psm2_kin__.FK( self.__psm2_last_jnt__)
        
        if self.__mode__ == self.MODE.hardware:
            self.__hw_mtml__.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
            self.__hw_mtml__.set_wrench_body_force([0,0,0])
            self.__hw_mtml__.set_gravity_compensation(True)
            
            self.__hw_mtmr__.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
            self.__hw_mtmr__.set_wrench_body_force([0,0,0])
            self.__hw_mtmr__.set_gravity_compensation(True)
        
        
#         self.__align_mtms_to_psms__()
        
#         self.__hw_mtmr__.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
#         self.__lock_mtml_psm2_translation__.publish(Bool(False))
#         self.__lock_mtml_psm2_orientation__.publish(Bool(False))
#         self.__lock_mtmr_psm1_translation__.publish(Bool(False))
#         self.__lock_mtmr_psm1_orientation__.publish(Bool(False))
        
    def disable_teleop(self):
        """!
        Disable teleoperation
        """
        
        self.__reenable_teleop__ = False
        self.__enabled__ = False

        if self.__mode__ == self.MODE.hardware:
            self.__last_mtml_gripper__ = self.__mtml_gripper__
            self.__last_mtmr_gripper__ = self.__mtmr_gripper__
            
            self.__hw_mtml__.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
            self.__hw_mtml__.set_gravity_compensation(False)
            
            self.__hw_mtmr__.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
            self.__hw_mtmr__.set_gravity_compensation(False)
        
#         self.__lock_mtml_psm2_translation__.publish(Bool(True))
#         self.__lock_mtml_psm2_orientation__.publish(Bool(True))
#         self.__lock_mtmr_psm1_translation__.publish(Bool(True))
#         self.__lock_mtmr_psm1_orientation__.publish(Bool(True))
    
    def rotate(self, axis, angle):
        """!
        Returns a rotation matrix and a transformation matrix
            @param axis : 'x','y' or 'z'
            @param angle : In radians
            @return r : a 3x3 rotation matrix
            @return t : a 4x4 transformation matrix
        """
        c = np.cos(angle)
        s = np.sin(angle)
        
        r = np.eye(3)
        
        if axis.lower() == 'x':
            r[1:,1:] = np.matrix( [ [c,-s], [s,c]])
        elif axis.lower() == 'z':
            r[:2,:2] = np.matrix( [ [c,-s], [s,c]])
        elif axis.lower() == 'y':
            r[0,0] = c; r[0,2] = s; r[2,0] = -s; r[2,2] = c;
        t = np.eye(4,4)
        t[0:3, 0:3] = r    
        return r, t

    
    def lock_mtm_orientations(self):
        """!
        Lock the orientations of MTMs
        """
        mtml_pose = self.__mtml_kin__.forward(self.__last_mtml_jnt__)
        mtmr_pose = self.__mtmr_kin__.forward(self.__last_mtmr_jnt__)
        mtml_quat = pose_converter.PoseConv.to_pos_quat(mtml_pose)
        mtmr_quat = pose_converter.PoseConv.to_pos_quat(mtmr_pose)
#              
        ql = Quaternion(); ql.w, ql.x, ql.y, ql.z = mtml_quat[1]
        qr = Quaternion(); qr.w, qr.x, qr.y, qr.z = mtmr_quat[1]
        
        if self.__mode__ == self.MODE.hardware: 
            self.__pub_lock_mtml_orientation__.publish(ql)
            self.__pub_lock_mtmr_orientation__.publish(qr)
        
    def __clutch_cb__(self, msg):
        """!
        Handle the repositioning clutch
        """
        if msg.buttons[0] == 1 and self.__enabled__:
            if self.__clutch_active__ == False:
                self.__clutch_active__ = True
                self.disable_teleop()
            
            self.lock_mtm_orientations()
            
        elif self.__clutch_active__ == True:
            self.__clutch_active__ = False
            self.__reenable_teleop__ = False
            self.enable_teleop()
            
            
            
            
    def __headsensor_cb__(self, msg):
        """!
        Callback for the head sensor
        """
        if msg.buttons[0] == 1:
            self.enable_teleop()            
        else:
            self.disable_teleop()
            
    
    # camera info callback
    def __get_cam_info(self, msg):
        if msg.header.frame_id == '/fake_cam_left_optical_link':
            self.__cam_info__['left'] = msg
        elif msg.header.frame_id == '/fake_cam_right_optical_link':
            self.__cam_info__['right'] = msg
            
    def adjust_ecm_pos(self):
#         self.disable_teleop()
        
        # No autocamera without teleop
        if self.__enabled__ == False:
            return
        
        j = lambda x, n : JointState(position = x, name = n) # convert to JointState message
        joint_angles = {'ecm': j(self.__ecm_last_jnt__, self.__ecm_joint_names__), 'psm1': j(self.__psm1_last_jnt__, self.__psm1_joint_names__), 'psm2': j(self.__psm2_last_jnt__, self.__psm2_joint_names__)}
        if None not in joint_angles.values():
            self.set_ecm_to_world_transform(self.__T_ecm__)
            self.__autocamera__.set_method(2)

            # The name of the frame we want to show the deadzone in            
            frame_name = '/world'
            
            p = self.__autocamera__.get_3d_deadzone(self.__cam_info__, frame_name)
            
            self.__deadzone_pub__.publish(p)
            
            jnt_msg = self.__autocamera__.compute_viewangle(joint_angles, self.__cam_info__)
            self.__pub_ecm__.publish(jnt_msg)
            self.move_arm_joints('ecm', jnt_msg.position) 

    def __ecm_cb__(self, msg):
        """!
        Store the ECM joint angles and 
        end-effector position every time a new message is received
        """
        if self.__mode__ == self.MODE.simulation:
            self.__ecm_last_jnt__ = msg.position[0:2] + msg.position[-2:]
        elif self.__mode__ == self.MODE.hardware:
            self.__ecm_last_jnt__ = msg.position[0:3] + tuple([0])
        
        self.__T_ecm__ = self.__ecm_kin__.forward(self.__ecm_last_jnt__)
          
    def __psm1_cb__(self, msg):
        """!
        Store PSM joint angles, and move the simulation if the hardware is active
        """
        if self.__mode__ == self.MODE.simulation:
            p = msg.position
            msg.position = [p[0], p[1], p[7], p[8], p[9], p[10], p[11]]
            
        msg.name =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
        if self.__psm1_first_pos__ is None:
            self.__psm1_first_pos__, _ = self.__psm1_kin__.FK( msg.position[0:-1] )
            
        self.__psm1_last_jnt__ = msg.position[0:-1]
        if self.__mode__ == self.MODE.hardware:
            self.__pub_psm1__.publish(msg)
            
        self.adjust_ecm_pos()
            
        
    def __psm2_cb__(self, msg):
        """!
        Store PSM joint angles, and move the simulation if the hardware is active
        """
        
        if self.__mode__ == self.MODE.simulation:
            p = msg.position
            msg.position = [p[0], p[1], p[7], p[8], p[9], p[10], p[11]]
            
        msg.name =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
        if self.__psm2_first_pos__ is None:
            self.__psm2_first_pos__, _ = self.__psm2_kin__.FK( msg.position[0:-1] )
            
        self.__psm2_last_jnt__ = msg.position[0:-1]
        if self.__mode__ == self.MODE.hardware:
            self.__pub_psm2__.publish(msg)
            
    
    def __mtml_gripper_cb__(self, msg):
        """!
        Record position of the gripper
        """
        self.__mtml_gripper__ = msg.data
        
                
    def __mtmr_gripper_cb__(self, msg):
        self.__mtmr_gripper__ = msg.data
                
    def __mtml_cb__(self, msg):
        """!
        The main part of the teleoperation is performed in this
        callback function. 
        """
        # Find mtm end effector position and orientation
#         self.__align_mtms_to_psms__()
        if self.__ecm_last_jnt__ is None: return
        if self.__mode__ == self.MODE.simulation:
            msg.position = msg.position[0:2] + msg.position[3:] 
            msg.name = msg.name[0:2] + msg.name[3:]
        else:
            msg.position = msg.position[0:-1]
            msg.name = msg.name[0:-1]
#         msg.position = [.8 * i for i in msg.position]
        
        self.__last_mtml_jnt__ = msg.position
        if self.__mtml_first_pos__ is None:
            self.__mtml_first_pos__, _ = self.__mtml_kin__.FK( msg.position)
            
        if self.__T_mtml_000__ == None :
            self.__T_mtml_000__ = self.__mtml_kin__.forward(msg.position)

        # These rotations help the robot move better
        _, r_330_y_t = self.rotate('y', -np.pi/6.0)
        _, r_330_z_t = self.rotate('z', -np.pi/6.0) 
        _, r_30_x_t = self.rotate('x', np.pi/6.0)
        
        T_mtm = self.__mtml_kin__.forward(msg.position)
        T = ( self.__T_mtml_000__**-1) * T_mtm 
        T =  r_330_z_t * T * r_330_y_t * r_30_x_t
        transform = np.matrix( [ [0,-1,0,0], 
                                [0,0,1,0], 
                                [-1,0,0,0], 
                                [0,0,0,1]])
        T = transform * T
        

        
  
        pos = T[0:3,3]
        rot = T[0:3,0:3]
        
        if self.__mtml_home_position__ is None:
            self.__mtml_home_position__ = pos
        
        if self.__mtml_last_pos__ == None or self.__clutch_active__:
            self.__mtml_first_pos__ = pos
            self.__mtml_last_pos__ = pos
            self.__mtml_last_rot__ = rot
            return
        
        
        
        if self.__enabled__ == False: return
        if self.__mode__ == self.MODE.hardware:
            self.__mtml_wrist_adjustment__.publish()
        
        delta = pos - self.__mtml_first_pos__
        delta = np.insert(delta, 3,1).transpose()
        p0 = np.insert(self.__mtml_first_pos__, 3,1).reshape(4,1)
        p1 = np.insert(self.__mtml_last_pos__, 3,1,).reshape(4,1)
        p2 = np.insert(pos, 3,1).transpose().reshape(4,1)
        
        movement = p2-p1
        sx,sy,sz = self.__get_dynamic_scale('mtml')
        movement[0] *= sx
        movement[1] *= sy
        movement[2] *= sz
        if self.__mtml_translations__ is None:
            self.__mtml_translations__ = (p1-p0) + movement
        else:
            self.__mtml_translations__ += movement
            
        new_translation = self.__mtml_translations__
#         translation = (p1-p0)
        translation = ( self.__T_ecm__ * new_translation)[0:3]
        
        orientation = self.__T_ecm__[0:3,0:3] * rot

        T = self.__translate_mtml__(translation)
        T = self.__set_orientation_mtml__( orientation, T)
        
        q = list(self.__psm2_last_jnt__)
        q[5] = 0
        q[4] = 0
        q[3] = 0
        new_psm2_angles = self.__psm2_kin__.inverse(T, q)
        
#         print("T = {} , \nnew_psm2_angles = {}\n\n".format(T, new_psm2_angles))
        
        if (new_psm2_angles is None): return
        
        
#         if type(new_psm2_angles) == NoneType:
#             print("Frozen, Translation = " + translation.__str__())
# #             self.__mtml_first_pos__ = self.__mtml_last_pos__
#             T = self.__set_orientation_mtml__( self.__last_good_psm2_transform__[0:3,0:3] ) 
#             new_psm2_angles = self.__psm1_kin__.inverse(T, q)
        if self.__mode__ == self.MODE.hardware:    
            if type(new_psm2_angles) == NoneType:
                self.__hw_mtml__.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
                self.__hw_mtml__.set_gravity_compensation(False)
                self.__pub_unlock_mtml_orientation__.publish()
                return
            else:
                self.__hw_mtml__.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
                self.__hw_mtml__.set_wrench_body_force([0, 0, 0])
                self.__hw_mtml__.set_gravity_compensation(True)
            
        self.__last_good_psm2_transform__ = T
        
        if self.__mode__ == self.MODE.hardware:
            gripper = (self.__mtml_gripper__-.4) * 1.4/.6
            new_psm2_angles = np.append(new_psm2_angles, gripper)
            
        if self.__mode__ == self.MODE.hardware:
            self.__hw_psm2__.move_joint_list( new_psm2_angles.tolist(), range(0,len(new_psm2_angles)), interpolate=False)
          
        if self.__mode__ == self.MODE.simulation:
            msg = JointState()
            msg.position = new_psm2_angles.tolist()
            msg.name =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
            self.__pub_psm2__.publish(msg)
#         self.move_arm_joints('psm2', new_psm2_angles.tolist())
            
        # Find the direction of mtml movement
        self.__mtml_dir__ = pos - self.__mtml_last_pos__
        self.__mtml_last_pos__ = pos
        self.__mtml_last_rot__ = rot
        
        
        
    
    def __mtmr_cb__(self, msg):
        # Find mtm end effector position and orientation
        if self.__ecm_last_jnt__ is None: return
        
        if self.__mode__ == self.MODE.simulation:
            msg.position = msg.position[0:2] + msg.position[3:] 
            msg.name = msg.name[0:2] + msg.name[3:]
        else:
            msg.position = msg.position[0:-1]
            msg.name = msg.name[0:-1]

#         msg.position = [.8 * i for i in msg.position]
        self.__last_mtmr_jnt__ = msg.position
        if self.__mtmr_first_pos__ is None:
            self.__mtmr_first_pos__, _ = self.__mtml_kin__.FK( msg.position )
            
        if self.__T_mtmr_000__ == None :
            self.__T_mtmr_000__ = self.__T_mtml_000__
        
        # These rotations help the robot move better
        _, r_330_y_t = self.rotate('y', -np.pi/6.0)
        _, r_330_z_t = self.rotate('z', -np.pi/6.0)
        _, r_330_x_t = self.rotate('x', -np.pi/6.0)
        
        T_mtm = self.__mtmr_kin__.forward(msg.position)
        
        T = ( self.__T_mtmr_000__**-1) * T_mtm 
        T = r_330_z_t * T * r_330_y_t * r_330_x_t
        
        transform = np.matrix( [ [0,-1,0,0], 
                                [0,0,1,0], 
                                [-1,0,0,0], 
                                [0,0,0,1]])
        T = transform * T
        
        rot = T[0:3,0:3]
        pos = T[0:3,3]
        
        if self.__mtmr_home_position__ is None:
            self.__mtmr_home_position__ = pos
        
        if self.__mtmr_last_pos__ == None  or self.__clutch_active__:
            self.__mtmr_first_pos__ = pos
            self.__mtmr_last_pos__ = pos
            self.__mtmr_last_rot__ = rot
            return
        
        # Find the direction of mtmr movement
        self.__mtmr_dir__ = pos - self.__mtmr_last_pos__
        
        self.__mtmr_last_pos__ = pos
        self.__mtmr_last_rot__ = rot
        
        if self.__enabled__ == False: return
        
        if self.__mode__ == self.MODE.hardware:
            self.__mtmr_wrist_adjustment__.publish()
        
        delta = pos - self.__mtmr_first_pos__
        delta = np.insert(delta, 3,1).transpose()
        p0 = np.insert(self.__mtmr_first_pos__, 3,1).reshape(4,1)
        p1 = np.insert(pos, 3,1).transpose().reshape(4,1)
        
        translation = (p1-p0)
        translation = ( self.__T_ecm__ * translation)[0:3]
        
        orientation = self.__T_ecm__[0:3,0:3] * rot
        
        T = self.__translate_mtmr__(translation)
        T = self.__set_orientation_mtmr__(orientation, T)
        
        q = list(self.__psm1_last_jnt__)
        q[5] = 0
        q[4] = 0
        q[3] = 0
        new_psm1_angles = self.__psm1_kin__.inverse(T, q)
#         print("T = {} , \nnew_psm1_angles = {}\n\n".format(T, new_psm1_angles))
        
        if (new_psm1_angles is None): return
        
#         if type(new_psm1_angles) == NoneType:
#             T[0:3, 0:3] = self.__last_good_psm1_transform__[0:3,0:3] 
#             new_psm1_angles = self.__psm1_kin__.inverse(T, q)
        if self.__mode__ == self.MODE.hardware:            
            if type(new_psm1_angles) == NoneType:
                self.__hw_mtmr__.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
                self.__hw_mtmr__.set_gravity_compensation(False)
                self.__pub_unlock_mtmr_orientation__.publish()
                return
            else:
                self.__hw_mtmr__.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
                self.__hw_mtmr__.set_wrench_body_force([0, 0, 0])
                self.__hw_mtmr__.set_gravity_compensation(True)
        
#         self.__last_good_psm1_transform__ = T
        
        if self.__mode__ == self.MODE.hardware:
            gripper = (self.__mtmr_gripper__-.4) * 1.2/.4
            new_psm1_angles = np.append(new_psm1_angles, gripper)
                
        if self.__mode__ == self.MODE.hardware:
            self.__hw_psm1__.move_joint_list( new_psm1_angles.tolist(), range(0,len(new_psm1_angles)), interpolate=False)
         
        if self.__mode__ == self.MODE.simulation:
            msg = JointState()
            msg.position = new_psm1_angles.tolist()
            msg.name =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
            self.__pub_psm1__.publish(msg)
#         self.move_arm_joints('psm1', new_psm1_angles.tolist())
        
        
    
    def __align_mtms_to_psms(self):
        """!
        This function should align the orientations of mtms to the psms. This is not completed yet.
        """
        T_psm1 = self.__psm1_kin__.forward(self.__psm1_last_jnt__)
        T_psm2 = self.__psm2_kin__.forward(self.__psm2_last_jnt__)
        
        T_mtml = self.__mtml_kin__.forward(self.__last_mtml_jnt__)
        T_mtmr = self.__mtmr_kin__.forward(self.__last_mtmr_jnt__)
        T_mtml_000 = self.__mtml_kin__.forward([0,0,0,0,0,0,0])
        
        T_mtml[0:3,0:3] = (((self.__T_ecm__ * (T_mtml_000 ** -1)) ** -1) * T_psm2)[0:3,0:3]
        T_mtmr[0:3, 0:3] = T_psm1[0:3, 0:3]
        
        jnt_mtml = self.__mtml_kin__.inverse(T_mtml)
        jnt_mtmr = self.__mtmr_kin__.inverse(T_mtmr)
        print('aligning')
        if self.__mode__ == self.MODE.hardware:
            self.__hw_mtml__.move_joint_list( jnt_mtml.tolist(), range(0, len(jnt_mtml)), interpolate=True)
            self.__hw_mtmr__.move_joint_list( jnt_mtmr.tolist(), range(0, len(jnt_mtmr)), interpolate=True)
            
        
    
    def __get_dynamic_scale(self, arm_name):
        if arm_name.lower() == 'mtml':
            home_position = self.__mtml_home_position__
            dir = self.__mtml_dir__
            pos = self.__mtml_last_pos__
        elif arm_name.lower() == 'mtmr':
            home_position = self.__mtmr_home_position__
            dir = self.__mtmr_dir__
            pos = self.__mtmr_last_pos__
            
        if home_position is None:
            return self.scale, self.scale, self.scale
        
        dir = [float(i) for i in dir]
        h = [float(i) for i in (pos-home_position)]
        
        dir = [ i/abs(i) if i !=0 else i for i in dir]
        loc_vec = [ i/abs(i) if i!=0 else i for i in h]

        # positive means moving away from center 
        # and negative means moving towards it
        # If it's moving away we want to increase
        # the scaling and if it's moving closer to center
        # we want to decrease the scaling
        signs = [i * j for i,j in zip(dir, loc_vec)]
        
#         mult = .2
#         self.__x_scale__ = self.__x_scale__ + h[0] * signs[0] * mult
#         self.__y_scale__ = self.__y_scale__ + h[1] * signs[1] * mult
#         self.__z_scale__ = self.__z_scale__ + h[2] * signs[2] * mult
        
        if signs[0] > 0: # moving away
            self.__x_scale__ = .3
        elif signs[0] <0:
            self.__x_scale__ = .3
        
        if signs[1] > 0: # moving away
            self.__y_scale__ = .3
        elif signs[1] <0:
            self.__y_scale__ = .3
             
        if signs[2] > 0: # moving away
            self.__z_scale__ = .3
        elif signs[2] <0:
            self.__z_scale__ = .3
        def set_max(x):
            m = .5
            if x > m:
                x = m
            return abs(x)
        self.__x_scale__ = set_max(self.__x_scale__)
        self.__y_scale__ = set_max(self.__y_scale__)
        self.__z_scale__ = set_max(self.__z_scale__)
        
#         print self.__x_scale__, self.__y_scale__, self.__z_scale__
        return self.__x_scale__, self.__y_scale__, self.__z_scale__
        
        
            
    def __translate_mtml__(self, translation, T=None): # translate a psm arm
#         if self.__enabled__ == False: return
#         
#         sx, sy, sz = self.__get_dynamic_scale('mtml')
#         print("scales = {}, {}, {}\n".format(sx,sy,sz))
#         
#         translation[0] = translation[0] * sx
#         translation[1] = translation[1] * sy
#         translation[2] = translation[2] * sz
        
        psm2_pos = self.__psm2_first_pos__#self.__psm2_kin__.FK(self.__psm2_last_jnt__)
        if T==None:
            T = self.__psm2_kin__.forward(self.__psm2_last_jnt__)
        new_psm2_pos = psm2_pos + translation
        T[0:3, 3] = new_psm2_pos
#         self.autocamera.add_marker(T, 'psm2_delta', color = [1,1,0], scale= [.02,0,0], type=Marker.LINE_LIST, points=[psm2_pos,new_psm2_pos], frame="world")
        return T
    
    def __translate_mtmr__(self, translation, T=None): # translate a psm arm
        if self.__enabled__ == False: return
        
        translation = translation * self.scale
        psm1_pos = self.__psm1_first_pos__ #self.__psm1_kin__.FK(self.__psm1_last_jnt__)
        
        if T==None:
            T = self.__psm1_kin__.forward(self.__psm1_last_jnt__)

        new_psm1_pos = psm1_pos + translation
        T[0:3, 3] = new_psm1_pos
        
        return T
        
    
    def __set_orientation_mtml__(self,orientation, T=None): # align a psm arm to mtm
        if self.__enabled__ == False: return
        
        if T==None:
            T = self.__psm2_kin__.forward(self.__psm2_last_jnt__)
        T[0:3,0:3] = orientation
        return T
        
    def __set_orientation_mtmr__(self,orientation, T = None): # align a psm arm to mtm
        if self.__enabled__ == False: return
        if T == None:
            T = self.__psm1_kin__.forward(self.__psm1_last_jnt__)
        T[0:3,0:3] = orientation
        return T
            
   
      