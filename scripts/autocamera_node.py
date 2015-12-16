#!/usr/bin/env python

import sys
import rospy
import autocamera_algorithm 
import os
import tf

from sensor_msgs.msg import JointState
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
jnt_msg = JointState()
joint_angles = {'ecm':None, 'psm1':None, 'psm2':None}
cam_info = None
    
def add_ecm_jnt(msg):
    add_jnt('ecm', msg)
def add_psm1_jnt(msg):
    global psm1_pub
    # We need to set the names, otherwise the simulation won't move
    msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']

    add_jnt('psm1', msg)
    psm1_pub.publish(msg)
    
def add_psm2_jnt(msg):
    global psm2_pub
    msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
    
    add_jnt('psm2', msg)
    psm2_pub.publish(msg)
    
def add_jnt(name, msg):
    global joint_angles, ecm_pub
    joint_angles[name] = msg
    if not None in joint_angles.values():
#         rospy.logerr(joint_angles.__str__())
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
    
    global ecm_pub, psm1_pub, psm2_pub
    
    rospy.init_node('autocamera_node')
    
    # Publishers to the simulation
    ecm_pub = rospy.Publisher('autocamera_node', JointState, queue_size=10)
    psm1_pub = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=10)
    psm2_pub = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=10)
    
    # Get the joint angles from the simulation
    rospy.Subscriber('/fakecam_node/camera_info', CameraInfo, get_cam_info)
    rospy.Subscriber('/dvrk_ecm/joint_states', JointState, add_ecm_jnt)
    
    # Get the joint angles from the hardware
    rospy.Subscriber('/dvrk/PSM1/position_joint_current', JointState, add_psm1_jnt)
    rospy.Subscriber('/dvrk/PSM2/position_joint_current', JointState, add_psm2_jnt)
    
#     rospy.Subscriber('/dvrk/PSM2/position_joint_current', JointState, add_psm2_jnt)
    
    rospy.spin()
    
    
if __name__ == "__main__":
    main()