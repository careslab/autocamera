#!/usr/bin/env python

import sys
import rospy
import autocamera_algorithm 
import os
import tf
import time
import numpy 

from robot import *
from sensor_msgs.msg import JointState
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics

jnt_msg = JointState()
joint_angles = {'ecm':None, 'psm1':None, 'psm2':None}
cam_info = None

last_ecm_jnt_pos = None

first_run = True

ecm_robot=None
psm1_robot=None

mtmr_robot=None
mrmr_kin=None
def ecm_manual_control():
#     if ecm_robot is None:
#         ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
#         ecm_kin = KDLKinematics(ecm_robot, ecm_robot.links[0].name, ecm_robot.links[-1].name)
#     if psm1_robot is None:
#         psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
#         psm1_kin = KDLKinematics(psm1_robot, psm1_robot.links[0].name, psm1_robot.links[-1].name)
    if mtmr_robot is None:
        mtmr_robot = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
        mtmr_kin = KDLKinematics(mtmr_robot, psm1_robot.links[0].name, psm1_robot.links[-1].name)
 
def move_psm1(msg):
    global psm1_hw
    jaw = msg.position[-3]
    msg = autocamera_algorithm.extract_positions(msg, 'psm1')
    
    msg.name = msg.name + ['jaw']
    msg.position = msg.position + [jaw]
    
    psm1_hw.move_joint_list(msg.position, interpolate=first_run)

def move_psm2(msg):
    global psm2_hw
    time.sleep(.5)
    jaw = msg.position[-3]
    msg = autocamera_algorithm.extract_positions(msg, 'psm2')
    
    msg.name = msg.name + ['jaw']
    msg.position = msg.position + [jaw]
    
    psm2_hw.move_joint_list(msg.position, interpolate=first_run)
    
def add_ecm_jnt(msg):
    global ecm_hw
    add_jnt('ecm', msg)
    
    
def add_psm1_jnt(msg):
    global psm1_pub
    # We need to set the names, otherwise the simulation won't move
    msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
    psm1_pub.publish(msg)
    add_jnt('psm1', msg)
        
     
    
def add_psm2_jnt(msg):
    global psm2_pub
    msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
    psm2_pub.publish(msg)
    add_jnt('psm2', msg)
        
def add_jnt(name, msg):
    global joint_angles, ecm_pub, ecm_hw, first_run
    joint_angles[name] = msg
    if not None in joint_angles.values():
        try:
            jnt_msg = 'error'
            jnt_msg = autocamera_algorithm.compute_viewangle(joint_angles, cam_info)
            t = time.time()
            jnt_msg.position = [ round(i,6) for i in jnt_msg.position]
            if len(jnt_msg.position) != 4 or len(jnt_msg.name) != 4 :
                return
            ecm_pub.publish(jnt_msg)
            
            #pos = jnt_msg.position; pos[2] = 0 # don't move the insertion joint at this stage 
            result = ecm_hw.move_joint_list(jnt_msg.position, interpolate=first_run)
            
            # Interpolate the insertion joint individually and the rest without interpolation
            #pos = jnt_msg.position; pos[0:2] = 0; pos[3] = 0
            #result = result * ecm_hw.move_joint_list(pos, interpolate=True)
            
            if result == True:
                first_run = False
#                 time.sleep(2)
                
            rospy.logerr(result.__str__())
            
            rospy.logerr(time.time() - t)
            
        except TypeError:
#             rospy.logerr('jnt_msg = ' + jnt_msg.__str__())
            pass
            
        joint_angles = dict.fromkeys(joint_angles, None)    
    

def get_cam_info(msg):
    global cam_info
    cam_info = msg      
        

def main():
    
    global ecm_pub, psm1_pub, psm2_pub, ecm_hw, psm1_hw, psm2_hw
    
    ecm_hw = robot('ECM')
    psm1_hw = robot('PSM1')
    psm2_hw = robot('PSM2')
    
    rospy.init_node('autocamera_node')
    
    # Publishers to the simulation
    ecm_pub = rospy.Publisher('autocamera_node', JointState, queue_size=10)
    psm1_pub = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=10)
    psm2_pub = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=10)
    
    # Get the joint angles from the simulation
    rospy.Subscriber('/fakecam_node/camera_info', CameraInfo, get_cam_info)
    rospy.Subscriber('/dvrk_ecm/joint_states', JointState, add_ecm_jnt)
    
    # Get the joint angles from the hardware and move the simulation from hardware
    rospy.Subscriber('/dvrk/PSM1/position_joint_current', JointState, add_psm1_jnt)
    rospy.Subscriber('/dvrk/PSM2/position_joint_current', JointState, add_psm2_jnt)
    
    
    # Move the hardware from the simulation
#     rospy.Subscriber('/dvrk_psm1/joint_states', JointState, move_psm1)
#     rospy.Subscriber('/dvrk_psm2/joint_states', JointState, move_psm2)
    
    rospy.spin()
    
    
if __name__ == "__main__":
    main()