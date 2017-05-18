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
from numpy import pi
import numpy as np
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PoseStamped

import tf
import std_msgs
from mercurial.util import interpolate

psm1_robot = None
psm1_kin = None

psm2_robot = None
psm2_kin = None

ecm_robot = None
ecm_kin = None

mtmr_robot = None
mtmr_kin = None

from robot import *

class mtm_aligner:
    def rotate(self, axis, angle):
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
    
    
    def ecm_joint_cb(self, msg):
        if msg.position and self.lets_do_it_once:
    #         lets_do_it_once = False
    #         q = msg.position[0:2] + msg.position[-2:]
            q = msg.position
            ecm_ee = self.ecm_kin.forward(q)
            ecm_base_frame = self.ecm_base.forward([]) 
            
            r_180_x = self.rotate('x', pi)
            r_90_z = self.rotate('z', -pi/2)
            
            ecm_ee[0:3,0:3] = ecm_ee[0:3,0:3] * r_90_z * r_180_x
            
            print("joint_angles " + q.__str__())
            print('ecm_ee' + ecm_ee.__str__())
            print('ecm_base' + ecm_base_frame.__str__())
            
            psm1_base_frame =  (ecm_ee ** -1) * self.psm1_kin.forward([]) 
            psm1_message = pose_converter.PoseConv.to_pose_msg(psm1_base_frame)
            psm1_message_stamped = pose_converter.PoseConv.to_pose_stamped_msg(psm1_base_frame)
        
            psm2_base_frame = (ecm_ee ** -1) * self.psm2_kin.forward([])
            psm2_message = pose_converter.PoseConv.to_pose_msg(psm2_base_frame)
            psm2_message_stamped = pose_converter.PoseConv.to_pose_stamped_msg(psm2_base_frame)
            
            self.psm1_pub.publish(psm1_message)
            self.psm2_pub.publish(psm2_message)
            
    #         mtmr_pub.publish(psm1_message)
    #         mtml_pub.publish(psm1_message)
            
    #         ecm_ee_inv = ecm_ee ** -1
    #         print('ecm_inverse' + ecm_ee_inv.__str__())
            ecm_message = pose_converter.PoseConv.to_pose_msg(ecm_base_frame)
    #         ecm_pub.publish(ecm_message)
            
            
            
            # Show it in RViz
            ecm_ee_message_stamped = pose_converter.PoseConv.to_pose_stamped_msg(ecm_ee)
            ecm_ee_message_stamped.header.frame_id = "/world"
            
            psm1_message_stamped.header.frame_id = "/world"
            self.tf_new_psm1_base.publish(psm1_message_stamped)
            
            psm2_message_stamped.header.frame_id = "/world"
            self.tf_new_psm2_base.publish(psm2_message_stamped)
            
        
        
    def __init__(self):
        a = rospy.init_node('set_base_frames')
        
        self.lets_do_it_once = True
            
        self.psm1_kin = None
        self.psm1_robot = None
        self.psm2_kin = None
        self.psm2_robot= None
        self.ecm_kin = None
        self.ecm_robot = None
        self.mtmr_robot = None
        self.mtmr_kin = None
        
        self.psm1_pub = None
        self.psm2_pub = None
        self.ecm_pub = None
        self.mtmr_pub = None
        self.mtml_pub = None
        self.ecm_base = None
        
        self.tf_new_psm2_base = None
        self.tf_new_psm1_base = None
        
        if self.psm1_robot is None:
            self.psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
            self.psm1_kin = KDLKinematics(self.psm1_robot, self.psm1_robot.links[0].name, self.psm1_robot.links[1].name)
        if self.psm2_robot is None:
            self.psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
            self.psm2_kin = KDLKinematics(self.psm2_robot, self.psm2_robot.links[0].name, self.psm2_robot.links[1].name)
        if self.ecm_robot is None:
            self.ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
            self.ecm_kin = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[-1].name)
            self.ecm_base = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[3].name)
        if self.mtmr_robot is None:
            self.mtmr_robot = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
            self.mtmr_base = KDLKinematics(self.mtmr_robot, self.mtmr_robot.links[0].name, self.mtmr_robot.links[1].name)
        
        rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.ecm_joint_cb)
        # pdb.set_trace()    
    
        self.psm1_pub = rospy.Publisher('/dvrk/PSM1/set_base_frame', Pose, latch=True, queue_size=1)
        self.psm2_pub = rospy.Publisher('/dvrk/PSM2/set_base_frame', Pose, latch=True, queue_size=1)
        
        self.mtmr_pub = rospy.Publisher('/dvrk/MTMR/set_base_frame', Pose, latch=True, queue_size=1)
        self.mtml_pub = rospy.Publisher('/dvrk/MTML/set_base_frame',Pose, latch=True, queue_size=1)
        
        self.mtmr_hw = robot("MTML")
        self.mtml_hw = robot("MTMR")
        
        self.psm1_hw = robot("PSM1")
        self.psm2_hw = robot("PSM2")
        
        
        self.ecm_pub = rospy.Publisher('/dvrk/ECM/set_base_frame', Pose, latch=True, queue_size=1)
         
        self.tf_new_psm1_base = rospy.Publisher('/new_psm1_base', PoseStamped, queue_size=10)
        self.tf_new_psm2_base = rospy.Publisher('/new_psm2_base', PoseStamped, queue_size=10)
         
#         psm1_base_frame = psm1_kin.forward([])
#         
#         psm1_message = pose_converter.PoseConv.to_pose_msg(psm1_base_frame)
#         
#         psm2_base_frame = psm2_kin.forward([]) 
#         psm2_message = pose_converter.PoseConv.to_pose_msg(psm2_base_frame)
#         
#         ecm_base_frame = ecm_base.forward([]) 
#         
#     #    ecm_message = pose_converter.PoseConv.to_pose_msg(np.linalg.inv(ecm_base_frame) )
#         ecm_message = pose_converter.PoseConv.to_pose_msg(ecm_base_frame)
        
    #     psm1_pub.publish(psm1_message)
    #     psm2_pub.publish(psm2_message)
    # #     mtmr_pub.publish(message)
    #     ecm_pub.publish(ecm_message)
        
#         from std_msgs.msg import Bool
#         self.mtml_gravity = rospy.Publisher('/dvrk/MTML/set_gravity_compensation', Bool, queue_size=10)
#         self.mtml_gravity.publish(False)
        
        self.psm1_hw.move_joint_list([0.0, 0.0, 0.0, 0.0], [3,4,5,6], interpolate=True)
        self.psm2_hw.move_joint_list([0.0, 0.0, 0.0, 0.0], [3,4,5,6], interpolate=True)
        
        self.mtml_hw.move_joint_list([1.57], [3], interpolate=True)
        self.mtmr_hw.move_joint_list([-1.58], [3], interpolate=True)
        
#         while True:
#             rospy.sleep(10)
#             self.mtml_hw.move_joint_list([1.5], [3], interpolate=True)
        
        rospy.spin()
    
 

    
if __name__ == "__main__":
    m = mtm_aligner()
    