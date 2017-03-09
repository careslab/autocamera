import scipy
import rospy
import shelve
import numpy as np

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from sensor_msgs.msg import JointState
from scipy.optimize import minimize 
from hrl_geom import pose_converter

class coregistrator:
    """
        The coregistrator class collects the joint angles from the da vinci platform and 
        optimizes the relative transformation of their bases. We have to touch the tips 
        of two pairs of arms together and record the data a few times. Then run the 
        optimization algorithm. We need to do it twice one for psm1 and psm2, and another
        one for psm1 and ecm.
    """
    
    class REGISTRATION_MODE:
        """
            The registration mode. 
        """
        PSM1_PSM2 = 'psm1_psm2'
        PSM1_ECM = 'psm1_ecm'
    
    # Static variables
    mode = REGISTRATION_MODE.PSM1_PSM2
    
    psm1_data = None
    psm2_data = None
    ecm_data = None
    
    def __init__(self):
        self.psm1_robot = None
        self.psm1_kin = None
        
        self.psm2_robot = None
        self.psm2_kin = None
        
        self.ecm_robot = None
        self.ecm_kin = None
    
        self.psm1_read_cb_save = False
        self.psm1_read_cb_count = 0
        
        self.psm2_read_cb_save = False
        self.psm2_read_cb_count = 0
        
        self.ecm_read_cb_save = False
        self.ecm_read_cb_count = 0
        
        self.collected_joint_angles = {}
        
        # Find the kinematic model of the arms from base to the end-effector
        # The base for psm1 and psm2 is link 1 but for ecm is link 3
        # The first link is the world
        if self.psm1_robot is None:
            self.psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
            self.psm1_kin = KDLKinematics(self.psm1_robot, self.psm1_robot.links[1].name, self.psm1_robot.links[-1].name)
        if self.psm2_robot is None:
            self.psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
            self.psm2_kin = KDLKinematics(self.psm2_robot, self.psm2_robot.links[1].name, self.psm2_robot.links[-1].name)
        if self.ecm_robot is None:
            self.ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
            self.ecm_kin = KDLKinematics(self.ecm_robot, self.ecm_robot.links[3].name, self.ecm_robot.links[-1].name)
    
        self.__init_nodes()
        
    def __init_nodes(self):
        
        rospy.init_node('psm_optimization_data_collector')
        # Get the joint angles from the hardware and move the simulation from hardware
        rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, self.psm1_read_cb)
        rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, self.psm2_read_cb)
        rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.ecm_read_cb)
    
    # Callback to the psm1 subscriber
    def psm1_read_cb(self, msg):
        if self.psm1_read_cb_save == True:
            rospy.loginfo("psm1 joint angles : " + msg.position.__str__())
            self.psm1_read_cb_save = False
            p, _ = self.psm1_kin.FK(msg.position[0:-1])
            
            if self.psm1_read_cb_count == 0 and not self.collected_joint_angles.has_key('psm1_angles'):
                self.collected_joint_angles["psm1_angles"] = []
                self.collected_joint_angles["psm1_xyz"] = []
            temp = self.collected_joint_angles["psm1_angles"];temp.append(msg.position); self.collected_joint_angles['psm1_angles'] = temp
            temp = self.collected_joint_angles["psm1_xyz"]; temp.append( p); self.collected_joint_angles["psm1_xyz"] = temp
            
            self.psm1_read_cb_count += 1
            
    
    
    # callback to the psm2 subscriber
    def psm2_read_cb(self, msg):
        if self.psm2_read_cb_save == True:
            rospy.loginfo("psm2 joint angles : " + msg.position.__str__())
            self.psm2_read_cb_save = False
            p, _ = self.psm2_kin.FK(msg.position[0:-1])
            
            if self.psm2_read_cb_count == 0 and not self.collected_joint_angles.has_key('psm2_angles'):
                self.collected_joint_angles["psm2_angles"] = []
                self.collected_joint_angles["psm2_xyz"] = []
            temp = self.collected_joint_angles["psm2_angles"];temp.append(msg.position); self.collected_joint_angles['psm2_angles'] = temp
            temp = self.collected_joint_angles["psm2_xyz"]; temp.append( p); self.collected_joint_angles["psm2_xyz"] = temp
            
            self.psm2_read_cb_count += 1
    
    # ecm callback
    def ecm_read_cb(self, msg):
        if self.ecm_read_cb_save == True:
            rospy.loginfo("ecm joint angles : " + msg.position.__str__())
            self.ecm_read_cb_save = False
            p, _ = self.ecm_kin.FK(msg.position)
            
            if self.ecm_read_cb_count == 0 and not self.collected_joint_angles.has_key('ecm_angles'):
                self.collected_joint_angles["ecm_angles"] = []
                self.collected_joint_angles["ecm_xyz"] = []
            temp = self.collected_joint_angles["ecm_angles"];temp.append(msg.position); self.collected_joint_angles['ecm_angles'] = temp
            temp = self.collected_joint_angles["ecm_xyz"]; temp.append( p); self.collected_joint_angles["ecm_xyz"] = temp
            
            self.ecm_read_cb_count += 1
    
        
    def collect(self):
        while True:
            print("save now? ")
            print("(y) yes\n(n) no\n(q) quit")
            r = raw_input(" : ")
            
            if r == "q":
                return
            if r == "y":
                self.psm1_read_cb_save = True
                self.psm2_read_cb_save = True
                self.ecm_read_cb_save = True
    
    def save_to_db(self, db_name):
        file = shelve.open(db_name)
        file.update(self.collected_joint_angles);
        file.close()
    
    def read_from_db(self, db_name):
        file = shelve.open(db_name)
        self.collected_joint_angles = file


    def compute_fk(self, name):
        if name == "psm1":
            psm1_joint_angles = self.collected_joint_angles['psm1_angles']
            data = []
            for angles in psm1_joint_angles:
                xyz,_ = self.psm1_kin.FK(angles[:-1])
                data.append(xyz)
            return data
        if name == "psm2":
            psm2_joint_angles = self.collected_joint_angles['psm2_angles']
            data = []
            for angles in psm2_joint_angles:
                xyz,_ = self.psm2_kin.FK(angles[:-1])
                data.append(xyz)
            return data
        if name == "ecm":
            psm2_joint_angles = self.collected_joint_angles['ecm_angles']
            data = []
            for angles in psm2_joint_angles:
                xyz,_ = self.ecm_kin.FK(angles)
                data.append(xyz)
            return data

    def dist(self, a,b):
        return np.sqrt( sum([ (i-j)**2 for i,j in zip(a,b)]))
    
    
    
    # This function is supposed to be fed to scipy
    def objective_function(self, xyzrpy):
        if len(xyzrpy) > 2:
            xyzrpy = [ tuple(xyzrpy[0:3]), tuple(xyzrpy[3:])]
        if self.psm1_data == None:
            self.psm1_data = self.compute_fk('psm1')
        if self.psm2_data == None:
            self.psm2_data = self.compute_fk('psm2')
        if self.ecm_data == None:
            self.ecm_data = self.compute_fk('ecm')
        
        T = pose_converter.PoseConv.to_homo_mat(xyzrpy)
        
        if self.mode == self.REGISTRATION_MODE.PSM1_ECM:
            pos_in_psm1rf = []
            for psm2_xyz in self.ecm_data:
                psm2_xyz = np.insert(psm2_xyz,3,1)
                psm2_xyz = psm2_xyz.transpose()
                temp = T *  psm2_xyz
                pos_in_psm1rf.append(temp)
        else:
            pos_in_psm1rf = []
            for psm2_xyz in self.psm2_data:
                psm2_xyz = np.insert(psm2_xyz,3,1)
                psm2_xyz = psm2_xyz.transpose()
                temp = T *  psm2_xyz
                pos_in_psm1rf.append(temp)
        
        diff = []
        for a,b in zip(self.psm1_data, pos_in_psm1rf):
            a = [ i[0] for i in a.tolist()]
            b = [ i[0] for i in b.tolist()]
            diff.append(self.dist(a,b))
        return sum(diff)/len(diff)
        
    
    def find_everything_related_to_world(self, arm_name, xyzrpy):
        if len(xyzrpy) > 2:
            xyzrpy = [ tuple(xyzrpy[0:3]), tuple(xyzrpy[3:])]
        
        psm1_kin_world_to_base = KDLKinematics(self.psm1_robot, self.psm1_robot.links[0].name, self.psm1_robot.links[1].name)
        Twp1 = psm1_kin_world_to_base.forward([])
                
        if arm_name == 'psm2':
            Tp12 = pose_converter.PoseConv.to_homo_mat(xyzrpy)
            
            Twp2 = Twp1 * Tp12
            Twp2_euler = pose_converter.PoseConv.to_pos_euler(Twp2)
            return Twp2_euler
        
        if arm_name == 'ecm':
            ecm_kin_sj_to_base = KDLKinematics(self.ecm_robot, self.ecm_robot.links[1].name, self.ecm_robot.links[3].name)
            Tse = ecm_kin_sj_to_base.forward([])
            Tp1E = pose_converter.PoseConv.to_homo_mat(xyzrpy)
            Tws = Twp1 * Tp1E * (Tse**-1)
            Tws_euler = pose_converter.PoseConv.to_pos_euler(Tws)
            
            return Tws_euler
    
    def optimize_bases(self):
        initial_guess = [ (.80,0.5,.3), (0.2,0.7,1.57)]
        res = minimize(self.objective_function, initial_guess, method='nelder-mead', options={'xtol':1e-12, 'disp':True, 'maxiter': 100000, 'maxfev':100000},)
        print(res)
        print(res.x)
        print(self.mode)   
        if self.mode == self.REGISTRATION_MODE.PSM1_PSM2:
            print('psm2 relative to world: ')
            v = self.find_everything_related_to_world('psm2', res.x)
     #       print("""xyz="{} {} {}" rpy="{} {} {}" """.format(v[0], v[1]) )
            print("""xyz="{0} {1} {2}" rpy="{3} {4} {5}" """.format(v[0][0],v[0][1],v[0][2],v[1][0],v[1][1],v[1][2]))
        if self.mode == self.REGISTRATION_MODE.PSM1_ECM:
            print('ecm relative to world: ')
            v = self.find_everything_related_to_world('ecm', res.x)
            print("""xyz="{0} {1} {2}" rpy="{3} {4} {5}" """.format(v[0][0],v[0][1],v[0][2],v[1][0],v[1][1],v[1][2]))
    

if __name__ == "__main__":
    c = coregistrator()
#     c.collect()
#     c.save_to_db("whatever.db")
    c.read_from_db("joint_angles.db_10_19_16")
    
    print("\n\n\nPSM1_ECM \n")
    c.mode = c.REGISTRATION_MODE.PSM1_ECM
    c.optimize_bases()
    
    print("\n\n\nPSM1_PSM2\n")
    c.mode = c.REGISTRATION_MODE.PSM1_PSM2
    c.optimize_bases()
