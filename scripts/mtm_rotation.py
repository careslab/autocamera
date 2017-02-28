import rospy
import shelve
import numpy as np
import math
# import pdb

from scipy.optimize import minimize 
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from sensor_msgs.msg import JointState
from hrl_geom import pose_converter
from mpl_toolkits.axisartist import angle_helper
from geometry_msgs.msg import Pose, Quaternion
from rospy.timer import sleep


psm1_robot = None
psm1_kin = None

psm2_robot = None
psm2_kin = None

ecm_robot = None
ecm_kin = None

def rotate(axis, angle):
    """
    Returns a rotation matrix
        axis : 'x','y' or 'z'
        angle : In radians
    """
    c = np.cos(angle)
    s = np.sin(angle)
    
    m = np.eye(3)
    
    if axis.lower() == 'x':
        m[1:,1:] = np.matrix( [ [c,-s], [s,c]])
    elif axis.lower() == 'z':
        m[:2,:2] = np.matrix( [ [c,-s], [s,c]])
    elif axis.lower() == 'y':
        m[0,0] = c; m[0,2] = s; m[2,0] = -s; m[2,2] = c;
    return m
def pose_from_homomat(homomat):
    my_pose = pose_converter.PoseConv.to_pose_msg(homomat)
    #my_pose = Pose() 
    #my_pose.position = my_quat[0]; 
    #my_pose.orientation = my_quat[1]
    return my_pose
    
def quat_from_homomat(homomat):
    my_quat = pose_converter.PoseConv.to_pos_quat(homomat)
    return my_quat[1]

def main():
    rospy.init_node('set_base_frames')
    sleep (1)
    global psm1_kin, psm1_robot, psm2_kin, psm2_robot, ecm_kin, ecm_robot
    if psm1_robot is None:
        psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        psm1_kin = KDLKinematics(psm1_robot, psm1_robot.links[0].name, psm1_robot.links[1].name)
    if psm2_robot is None:
        psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
        psm2_kin = KDLKinematics(psm2_robot, psm2_robot.links[0].name, psm2_robot.links[1].name)
    if ecm_robot is None:
        ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        ecm_kin = KDLKinematics(ecm_robot, ecm_robot.links[0].name, ecm_robot.links[-1].name)
        ecm_base = KDLKinematics(ecm_robot, ecm_robot.links[0].name, ecm_robot.links[3].name)
   
    # pdb.set_trace()    
    
    mtml_pub = rospy.Publisher('/dvrk/MTML/set_base_frame', Pose, queue_size=1)
    mtmr_pub = rospy.Publisher('/dvrk/MTMR/set_base_frame', Pose, queue_size=1)
    ecm_pub = rospy.Publisher('/dvrk/ECM/set_base_frame', Pose, queue_size=1)
    psm1_pub = rospy.Publisher('/dvrk/PSM1/set_base_frame', Pose, queue_size=1)
    psm2_pub = rospy.Publisher('/dvrk/PSM2/set_base_frame', Pose, queue_size=1)
    
    mtmr_psm1 = rospy.Publisher('/dvrk/MTMR_PSM1/set_registration_rotation', Quaternion, queue_size=1)
    mtml_psm2 = rospy.Publisher('/dvrk/MTML_PSM2/set_registration_rotation', Quaternion, queue_size=1)
        
    
    p1_base = psm1_kin.forward([]) # PSM1 Base Frame
    p2_base = psm2_kin.forward([]) # PSM2 Base Frame
    e_base = ecm_base.forward([]) # ECM Base Frame
    
    
    e = ecm_kin.forward([0,0,0,0]) # ECM Tool Tip
    
    camera_view_transform = pose_converter.PoseConv.to_homo_mat( [(0.0,0.0,0.0), (1.57079632679, 0.0, 0.0)])
    
    r = lambda axis, rad: rotate(axis, rad)
    
    mtmr_m = e;# mtmr_m = mtmr_m**-1 
    
    mtml_m = e

    print 'qmat'
    qmsg = Quaternion()
    temp = quat_from_homomat(p1_base)
    print quat_from_homomat(p1_base)
    
    message = pose_from_homomat(p1_base);
    psm1_pub.publish(message)
    while not rospy.is_shutdown():
        #print p1_base
        #print message
        psm1_pub.publish(message)
        print ('sure\n')
        
        
#     psm2_pub.publish(pose_from_homomat(p2_base))
#     psm1_pub.publish(pose_from_homomat(e_base))
#     mtml_pub.publish( pose_from_homomat(mtml_m))
#     mtmr_pub.publish( pose_from_homomat(mtmr_m))
    
           
    print ('\n\Hello:  nmtml rotation: \n')
    print( mtml_m.__repr__( ))
    print(pose_from_homomat(mtml_m)) 
    print ('\n\nmtmr rotation: \n')
    print( mtmr_m.__repr__())
    #rospy.spin();

    
if __name__ == "__main__":
    main()