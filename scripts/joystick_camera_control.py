from __common_imports__ import *
class JoystickClass:
    class MODE:
        """
            simulation mode: Use the hardware for the master side, 
                            and simulation for the ECM
            hardware mode: Use hardware for both the master side,
                            and the ECM
        """
        simulation = "SIMULATION"
        hardware = "HARDWARE"
        
    RAKE_CONTROL = "RAKE_CONTROL"
    ABSOLUTE_CONTROL = "ABSOLUTE_CONTROL"
    
    def __init__(self, mode = MODE.simulation):        
        self.__mode__ = mode
        self.joint_angles = []
        self.center = [0,0,0,0]
        self.joystick_at_zero = True
        self.movement_scale = .2 
        self.last_z = 0
        self.control_mode = self.ABSOLUTE_CONTROL 
        
    def __init_nodes__(self):
        self.hw_ecm = robot("ECM")
        self.pub_ecm_sim = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=10)
        self.__ecm_robot__ = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        self.__ecm_kin__ = KDLKinematics(self.__ecm_robot__, self.__ecm_robot__.links[0].name, self.__ecm_robot__.links[-1].name)
        
        self.__sub_ecm__ = None
        if self.__mode__ == self.MODE.simulation:
            self.__sub_ecm__ = rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.__ecm_cb__)
        elif self.__mode__ == self.MODE.hardware:
            self.__sub_ecm__ = rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.__ecm_cb__)
#             self.hw_ecm.home()
            self.hw_ecm.move_joint_list([0.0,0.0,0.0,0.0], interpolate=True)
            
        self.sub_joy = rospy.Subscriber('/joy', msg.Joy, self.on_joystick_change_cb)
        
    def shutdown(self):
        print('shutting down joystick control')
        try:
            self.__sub_ecm__.unregister()
            self.sub_joy.unregister()
            self.pub_ecm_sim.unregister()
            self.hw_ecm.unregister()
            
            print( "Shutting down " + self.__class__.__name__)
        except e:
            print("couldn't unregister all the topics")

    def set_mode(self, mode):
        self.__mode__ = mode
                
    def spin(self):
        self.__init_nodes__()
        self.__spin__()
    def __spin__(self):
        rospy.spin()

    def __ecm_cb__(self, msg):
        if self.__mode__ == self.MODE.simulation:
            self.joint_angles = msg.position[0:2] + msg.position[-2:]
        elif self.__mode__ == self.MODE.hardware:
            self.joint_angles = msg.position[0:2] + (0,0)
        if self.last_z != 0.0:
            q = list(self.center)
            q = self.ecm_zoom(self.last_z, q)
            self.move_ecm([q[2]], index = [2])
    
    def ecm_joystick_recenter(self):
        self.center[0] = self.joint_angles[0]
        self.center[1] = self.joint_angles[1]
        self.center[3] = self.joint_angles[3]
        self.last_z = 0
        self.joystick_at_zero = False     
           
    def on_joystick_change_cb(self, message):
        j = msg.Joy()
        if sum( np.abs(message.axes[0:2])) != 0 and self.joystick_at_zero == False:
            return
        self.joystick_at_zero = True
        if len(message.buttons) == 0:
            message.buttons = [0]
        if len(message.axes) < 4:
            message.axes = tuple( [a for a in message.axes] + [1])
        if message.buttons[0] == 1:
                self.ecm_joystick_recenter()
                print('self.center = {}, self.joint_angles = {}'.format(self.center, self.joint_angles))
                
                
        elif message.axes[3] > .7 and self.joystick_at_zero == True: # The throttle 
            # Allow movement
            movement_vector = np.array(message.axes[0:2]) # left right and up down
                
            q = self.ecm_pan_tilt(movement_vector)
#             q = self.ecm_zoom(message.axes[-1], q)
            self.last_z = message.axes[-1]
            self.move_ecm([q[0], q[1],q[3]], index = [0,1,3]) 
    
    def ecm_zoom(self, z, q = []):
        if q == []:
            q = self.joint_angles
        if z == 0.0 :
            return q
        if z > 0 and q[2] < .21: # Zoom in
            q[2] = self.joint_angles[2] + .0002
        elif z < 0:
            q[2] = self.joint_angles[2] - .0002
#         if q[2] < 0 :
#             q[2] = .001
        q = [round(i,4) for i in q]
        q = [i+j for i,j in zip(self.center, q)]
#         self.joint_angles = q
        self.center[2] = q[2]
        return q
    
    def ecm_pan_tilt(self, movement_vector, q = []):
        q = []
        q = self.joint_angles
        if q:
            q = list(q)
            if self.control_mode == self.RAKE_CONTROL:
                q[0] += .4 * movement_vector[0] * self.movement_scale
                q[1] += .08 * movement_vector[1] * self.movement_scale
            
            elif self.control_mode == self.ABSOLUTE_CONTROL:
                q[0] = 1.0 * movement_vector[0] * self.movement_scale
                q[1] = 1.0 * movement_vector[1] * self.movement_scale
                
            q = [round(i,4) for i in q]
            q = [i+j for i,j in zip(self.center, q)]
#             self.move_ecm(q)
            return q
    
    # move ecm based on joint angles either in simulation or hardware
    def move_ecm(self, joint_angles, index = []):
        if self.__mode__ == self.MODE.simulation:
            msg = JointState()
            msg.position = joint_angles
            msg.name = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
            self.pub_ecm_sim.publish(msg)            
        elif self.__mode__ == self.MODE.hardware:
            if index == []:
                index = range(0,len(joint_angles))
            self.hw_ecm.move_joint_list(joint_angles, index, interpolate=False)
    
    def ecm_inverse(self, goal_pos):
        key_hole,_ = self.__ecm_kin__.FK([0,0,0,0])
        safe_angles = list(self.joint_angles)
        safe_angles[2] += 0.14
        
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
            rospy.logerr('error')
        if p != None:  
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
 