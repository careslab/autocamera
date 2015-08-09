#!/usr/bin/env python

import sys
import rospy
import autocamera_algorithm 
import os

from sensor_msgs.msg import JointState
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
jnt_msg = JointState()
joint_angles = {'ecm':None, 'psm1':None, 'psm2':None}
cam_info = None

def add_ecm_jnt(msg):
    add_jnt('ecm', msg)
def add_psm1_jnt(msg):
    add_jnt('psm1', msg)
def add_psm2_jnt(msg):
    add_jnt('psm2', msg)
def add_jnt(name, msg):
    global joint_angles, ecm_pub
    joint_angles[name] = msg
    
    if not None in joint_angles.values():
        try:
            jnt_msg = 'error'
            jnt_msg = autocamera_algorithm.compute_viewangle(joint_angles, cam_info)
            ecm_pub.publish(jnt_msg)
        except TypeError:
#             rospy.logerr('jnt_msg = ' + jnt_msg.__str__())
            pass
            
        joint_angles = dict.fromkeys(joint_angles, None)    
    

def get_cam_info(msg):
    global cam_info
    cam_info = msg      
        
def main():
    
    global ecm_pub
    rospy.init_node('autocamera_node')
    ecm_pub = rospy.Publisher('autocamera_node', JointState, queue_size=10)
    rospy.Subscriber('/fakecam_node/camera_info', CameraInfo, get_cam_info)
    rospy.Subscriber('/dvrk_ecm/joint_states', JointState, add_ecm_jnt)
    rospy.Subscriber('/dvrk_psm1/joint_states', JointState, add_psm1_jnt)
    rospy.Subscriber('/dvrk_psm2/joint_states', JointState, add_psm2_jnt)
    rospy.spin()
    
    
if __name__ == "__main__":
    main()