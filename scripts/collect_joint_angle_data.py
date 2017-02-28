import scipy
import rospy
import shelve

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from sensor_msgs.msg import JointState

psm1_robot = None
psm1_kin = None

psm2_robot = None
psm2_kin = None

ecm_robot = None
ecm_kin = None

file = shelve.open("joint_angles.db") 
def psm1_read_cb(msg):
    if psm1_read_cb.save == True:
        global file, psm1_robot, psm1_kin
        rospy.loginfo("psm1 joint angles : " + msg.position.__str__())
        psm1_read_cb.save = False
        p, _ = psm1_kin.FK(msg.position[0:-1])
        
        if psm1_read_cb.count == 0 and not file.has_key('psm1_angles'):
            file["psm1_angles"] = []
            file["psm1_xyz"] = []
        temp = file["psm1_angles"];temp.append(msg.position); file['psm1_angles'] = temp
        temp = file["psm1_xyz"]; temp.append( p); file["psm1_xyz"] = temp
        
        psm1_read_cb.count += 1
        
psm1_read_cb.save = False
psm1_read_cb.count = 0

def psm2_read_cb(msg):
    if psm2_read_cb.save == True:
        global file, psm2_robot, psm2_kin
        rospy.loginfo("psm2 joint angles : " + msg.position.__str__())
        psm2_read_cb.save = False
        p, _ = psm2_kin.FK(msg.position[0:-1])
        
        if psm2_read_cb.count == 0 and not file.has_key('psm2_angles'):
            file["psm2_angles"] = []
            file["psm2_xyz"] = []
        temp = file["psm2_angles"];temp.append(msg.position); file['psm2_angles'] = temp
        temp = file["psm2_xyz"]; temp.append( p); file["psm2_xyz"] = temp
        
        psm2_read_cb.count += 1
psm2_read_cb.save = False
psm2_read_cb.count = 0

def ecm_read_cb(msg):
    if ecm_read_cb.save == True:
        global file, ecm_robot, ecm_kin
        rospy.loginfo("ecm joint angles : " + msg.position.__str__())
        ecm_read_cb.save = False
        p, _ = ecm_kin.FK(msg.position)
        
        if ecm_read_cb.count == 0 and not file.has_key('ecm_angles'):
            file["ecm_angles"] = []
            file["ecm_xyz"] = []
        temp = file["ecm_angles"];temp.append(msg.position); file['ecm_angles'] = temp
        temp = file["ecm_xyz"]; temp.append( p); file["ecm_xyz"] = temp
        
        ecm_read_cb.count += 1
ecm_read_cb.save = False
ecm_read_cb.count = 0

def main():
    global save, psm1_robot, psm1_kin, psm2_robot, psm2_kin, ecm_robot, ecm_kin
    
    rospy.init_node('psm_optimization_data_collector')
    # Get the joint angles from the hardware and move the simulation from hardware
    rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, psm1_read_cb)
    rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, psm2_read_cb)
    rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, ecm_read_cb)
    
    if psm1_robot is None:
        psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        psm1_kin = KDLKinematics(psm1_robot, psm1_robot.links[0].name, psm1_robot.links[-1].name)
    if psm2_robot is None:
        psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
        psm2_kin = KDLKinematics(psm2_robot, psm2_robot.links[0].name, psm2_robot.links[-1].name)
    if ecm_robot is None:
        ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        ecm_kin = KDLKinematics(ecm_robot, ecm_robot.links[0].name, ecm_robot.links[-1].name)

    while True:
        print("save now? ")
        print("(y) yes\n(n) no\n(q) quit")
        r = raw_input(" : ")
        
        if r == "q":
            global file
            file.close()
            return
        if r == "y":
            psm1_read_cb.save = True
            psm2_read_cb.save = True
            ecm_read_cb.save = True
            
    rospy.spin()
    
            
    
if __name__ == "__main__":
    main()
    
