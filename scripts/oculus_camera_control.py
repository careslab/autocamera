from __common_imports__ import *

class OculusClass:
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
        self.joint_angles = []
        self.center = [0,0,0,0]
        self.joystick_at_zero = True
        self.movement_scale = .2 
        self.last_z = 0
        
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
            
        self.sub_oculus = rospy.Subscriber('/oculus', JointState, self.on_oculus_cb)
        
    def shutdown(self):
        print('shutting down oculus control')
        try:
            self.__sub_ecm__.unregister()
            self.sub_oculus.unregister()
            self.pub_ecm_sim.unregister()
            
            print( "Shutting down " + self.__class__.__name__)
        except Exception:
            print("couldn't unregister all the topics")

    def set_mode(self, mode):
        self.__mode__ = mode
                
    def spin(self):
        self.__init_nodes__()
        self.__spin__()
    def __spin__(self):
        rospy.spin()
        
    def on_oculus_cb(self, message):
#         if self.__mode__ == self.MODE.simulation:
        message.name = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
        self.pub_ecm_sim.publish(message)
        if self.__mode__ == self.MODE.hardware:
            print(message.__str__())
            self.hw_ecm.move_joint_list( list(message.position), interpolate=False)
            
    def __ecm_cb__(self, message):
        pass

