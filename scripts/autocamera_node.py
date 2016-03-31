#!/usr/bin/env python

import sys
import rospy
import autocamera_algorithm 
import os
import tf
import time
import numpy as np
import cv2
import cv_bridge

from robot import *
from sensor_msgs.msg import JointState
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from Crypto.Signature.PKCS1_PSS import PSS_SigScheme
import sensor_msgs

# move the actual ecm with sliders?
MOVE_ECM_WITH_SLIDERS = False

AUTOCAMERA_MODE = "SIMULATION" # "SIMULATION" or "HARDWARE"
# AUTOCAMERA_MODE = "HARDWARE" # "SIMULATION" or "HARDWARE"

jnt_msg = JointState()
joint_angles = {'ecm':None, 'psm1':None, 'psm2':None}
cam_info = None

last_ecm_jnt_pos = None

first_run = True

ecm_robot=None
psm1_robot=None

mtmr_robot=None
mtmr_kin=None

mtml_robot=None
mtml_kin=None

camera_clutch_pressed = False

DEBUG = True
def logerror(msg):
    if DEBUG:
        rospy.logerr(msg)
        
def ecm_manual_control_lock(msg, fun):
    if fun == 'ecm':
        ecm_manual_control_lock.ecm_msg = msg
    elif fun == 'mtml':
        ecm_manual_control_lock.mtml_msg = msg
    
    if ecm_manual_control_lock.mtml_msg != None and ecm_manual_control_lock.ecm_msg != None:
        ecm_manual_control(ecm_manual_control_lock.mtml_msg, ecm_manual_control_lock.ecm_msg)
        ecm_manual_control_lock.mtml_msg = None
        ecm_manual_control_lock.ecm_msg = None

        
ecm_manual_control_lock.mtml_msg = None
ecm_manual_control_lock.ecm_msg = None

def ecm_manual_control(mtml_msg, ecm_msg):
    # TODO: Find forward kinematics from mtmr and ecm. Move mtmr. Find the movement vector. Add it to the 
    # ecm position, use inverse kinematics and move the ecm.
    global ecm_robot, ecm_kin, mtmr_robot, mtmr_kin, mtml_robot, mtml_kin
    if ecm_robot is None:
        ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        ecm_kin = KDLKinematics(ecm_robot, ecm_robot.links[0].name, ecm_robot.links[-1].name)
#     if psm1_robot is None:
#         psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
#         psm1_kin = KDLKinematics(psm1_robot, psm1_robot.links[0].name, psm1_robot.links[-1].name)
#     if mtmr_robot is None:
#         mtmr_robot = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
#         mtmr_kin = KDLKinematics(mtmr_robot, mtmr_robot.links[0].name, mtmr_robot.links[-1].name)
    if mtml_robot is None:
        mtml_robot = URDF.from_parameter_server('/dvrk_mtml/robot_description')
        mtml_kin = KDLKinematics(mtml_robot, mtml_robot.links[0].name, mtml_robot.links[-1].name)
        
    global mtml_start_position, mtml_end_position, ecm_pub
    rospy.logerr("mtml" + mtml_kin.get_joint_names().__str__())
    start_coordinates,_ = mtml_kin.FK(mtml_start_position.position[:-1]) # Returns (position, rotation)
    end_coordinates,_ = mtml_kin.FK(mtml_msg.position[:-1])
    
    rospy.logerr(start_coordinates.__str__())
    rospy.logerr(end_coordinates.__str__())
    
    diff = np.subtract(end_coordinates, start_coordinates)
    rospy.logerr("diff = " + diff.__str__())
    
    # Find the ecm 3d coordinates, add the 'diff' to it, then do an inverse kinematics
    ecm_coordinates,_ = ecm_kin.FK(ecm_msg.position[0:2] + ecm_msg.position[-2:]) # There are a lot of excessive things here that we don't need
    ecm_pose = ecm_kin.forward(ecm_msg.position[0:2] + ecm_msg.position[-2:])
    
    # Figure out the new orientation and position to be used in the inverse kinematics
    b,_ = ecm_kin.FK([ecm_msg.position[0],ecm_msg.position[1],.14,ecm_msg.position[3]])
    keyhole, _ = ecm_kin.FK([0,0,0,0])
    ecm_current_direction = b-keyhole
    new_ecm_coordinates = np.add(ecm_coordinates, diff)
    ecm_new_direction = new_ecm_coordinates - keyhole
    r = autocamera_algorithm.find_rotation_matrix_between_two_vectors(ecm_current_direction, ecm_new_direction)
    
    ecm_pose[0:3,0:3] =  r* ecm_pose[0:3,0:3] 
    ecm_pose[0:3,3] = new_ecm_coordinates
    new_ecm_joint_angles = ecm_kin.inverse(ecm_pose)
    
    new_ecm_msg = ecm_msg; new_ecm_msg.position = new_ecm_joint_angles; new_ecm_msg.name = ecm_kin.get_joint_names()
    
    rospy.logerr("ecm_new_direction " + ecm_new_direction.__str__())
    rospy.logerr('ecm_coordinates' + ecm_coordinates.__str__())
    rospy.logerr("ecm_pose " + ecm_pose.__str__())
    rospy.logerr("new_ecm_joint_angles " + new_ecm_joint_angles.__str__())
    rospy.logerr("new_ecm_msg" + new_ecm_msg.__str__())
    
    ecm_pub.publish(new_ecm_msg)
    
    
def camera_clutch_cb(msg):
    global camera_clutch_pressed
    camera_clutch_pressed = msg.data
    rospy.logerr('Camera Clutch : ' + camera_clutch_pressed.__str__())
    
mtml_start_position = None
mtml_end_position = None
def mtml_cb(msg):
    global mtml_start_position, mtml_end_position
    if camera_clutch_pressed :
        if mtml_start_position == None:
            mtml_start_position = msg
        mtml_end_position = msg
        
        # move the ecm based on the movement of MTML
        ecm_manual_control_lock(msg, 'mtml')
    else:
        mtml_start_position = msg
        mtml_end_position = None

def move_psm1(msg):
    global psm1_hw
    jaw = msg.position[-3]
    msg = autocamera_algorithm.extract_positions(msg, 'psm1')
    
    msg.name = msg.name + ['jaw']
    msg.position = msg.position + [jaw]
    
    psm1_hw.move_joint_list(msg.position, interpolate=True)

def move_psm2(msg):
    global psm2_hw
    time.sleep(.5)
    jaw = msg.position[-3]
    msg = autocamera_algorithm.extract_positions(msg, 'psm2')
    
    msg.name = msg.name + ['jaw']
    msg.position = msg.position + [jaw]
    
    psm2_hw.move_joint_list(msg.position, interpolate=first_run)

# ecm callback    
def add_ecm_jnt(msg):
    if camera_clutch_pressed == False and msg != None:
        global ecm_hw, first_run
        if MOVE_ECM_WITH_SLIDERS == False:
            add_jnt('ecm', msg)
        else:
            temp = list(msg.position[:2]+msg.position[-2:])
            r = ecm_hw.move_joint_list(temp, interpolate=first_run)
            if r == True:
                first_run = False
    else:
        ecm_manual_control_lock(msg, 'ecm')
        
# psm1 callback    
def add_psm1_jnt(msg):
    if camera_clutch_pressed == False and msg != None:
        global psm1_pub
        # We need to set the names, otherwise the simulation won't move
        if AUTOCAMERA_MODE == "HARDWARE" :
            msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
            psm1_pub.publish(msg)
        add_jnt('psm1', msg)
            
     
# psm2 callback    
def add_psm2_jnt(msg):
    if camera_clutch_pressed == False and msg != None:
        global psm2_pub
        if AUTOCAMERA_MODE == "HARDWARE" :
            msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
            psm2_pub.publish(msg)
        add_jnt('psm2', msg)
            
def add_jnt(name, msg):
    global joint_angles, ecm_pub, ecm_hw, first_run
    joint_angles[name] = msg
    
    if not None in joint_angles.values():
        
        if initialize_psms.initialized>0:        
            initialize_psms()
            time.sleep(.01)
            initialize_psms.initialized -= 1
        
        try:
            jnt_msg = 'error'
            jnt_msg = autocamera_algorithm.compute_viewangle(joint_angles, cam_info)
            t = time.time()
            
            ecm_pub.publish(jnt_msg)
            jnt_msg.position = [ round(i,4) for i in jnt_msg.position]
            if len(jnt_msg.position) != 4 or len(jnt_msg.name) != 4 :
                return
            
            if AUTOCAMERA_MODE == "HARDWARE":
#                 pos = jnt_msg.position[0:2] + [jnt_msg.position[3] ]  # don't move the insertion joint at this stage 
                pos = jnt_msg.position
                result = ecm_hw.move_joint_list(pos, index=[0,1,2,3], interpolate=first_run)
                 
                # Interpolate the insertion joint individually and the rest without interpolation
                pos = [jnt_msg.position[2]]
                
#                 logerror('result = ' + result.__str__() + ', first_run = ' + first_run.__str__())
                if result:
                    first_run = False
#                 result = result * ecm_hw.move_joint_list(pos, index=[2], interpolate=True)
#              
            
#                 
        except TypeError:
#             rospy.logerr('Exception : ' + TypeError.message.__str__())
            pass
            
        joint_angles = dict.fromkeys(joint_angles, None)    
    

# camera info callback
def get_cam_info(msg):
    global cam_info
    cam_info = msg      
        

def left_image_cb( image_msg):
    global image_left_pub
    bridge = cv_bridge.CvBridge()
    
    im = bridge.imgmsg_to_cv2(image_msg, 'rgb8')
    
    l1 = autocamera_algorithm.find_zoom_level.positions['l1']; l1 = tuple(int(i) for i in l1)
    l2 = autocamera_algorithm.find_zoom_level.positions['l2']; l2 = tuple(int(i) for i in l2)
    lm = autocamera_algorithm.find_zoom_level.positions['lm']; lm = tuple(int(i) for i in lm)
    
    logerror('l1 = ' + l1.__str__())
    rotate_180 = lambda  p : (640-p[0], 480-p[2])

    cv2.circle(im, l1, 10, (0,255,0), -1)
    cv2.circle(im, l2, 10, (0,255,255), -1)
    cv2.circle(im, lm, 10, (0,0,255), -1)
    
    cv2.circle(im, (0,0), 20, (255,0,0), -1)
    cv2.circle(im, (640,480), 20, (255,0,255), -1)
    
    new_image = bridge.cv2_to_imgmsg(im, 'rgb8')

    new_image.header.seq = image_msg.header.seq
    new_image.header.stamp = image_msg.header.stamp
    new_image.header.frame_id = image_msg.header.frame_id
    
    image_left_pub.publish(new_image)
    
def right_image_cb( image_msg):
    pass


def initialize_psms():
    global psm1_pub, psm2_pub
    if AUTOCAMERA_MODE == "SIMULATION" : 
        msg = JointState()
        msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
        msg.position = [0.84 , -0.65, 0.10, 0.00, 0.00, 0.00, 0.00]
        psm1_pub.publish(msg)
        msg.position = [-0.84 , -0.53, 0.10, 0.00, 0.00, 0.00, 0.00]
        psm2_pub.publish(msg)
        rospy.logerr('psms initialized')
initialize_psms.initialized = 30

def main():
    
    global ecm_pub, psm1_pub, psm2_pub, ecm_hw, psm1_hw, psm2_hw, image_left_pub, image_right_pub
    
    ecm_hw = robot('ECM')
    psm1_hw = robot('PSM1')
    psm2_hw = robot('PSM2')
    
    rospy.init_node('autocamera_node')
    
    # Publishers to the simulation
    ecm_pub = rospy.Publisher('autocamera_node', JointState, queue_size=10)
    psm1_pub = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=10)
    psm2_pub = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=10)
    
    # Get the joint angles from the simulation
    rospy.Subscriber('/dvrk_ecm/joint_states', JointState, add_ecm_jnt)
    rospy.Subscriber('/fakecam_node/camera_info', CameraInfo, get_cam_info)
    
    if AUTOCAMERA_MODE == "HARDWARE" :
        # Get the joint angles from the hardware and move the simulation from hardware
        rospy.Subscriber('/dvrk/PSM1/position_joint_current', JointState, add_psm1_jnt)
        rospy.Subscriber('/dvrk/PSM2/position_joint_current', JointState, add_psm2_jnt)
    elif AUTOCAMERA_MODE == "SIMULATION":
        # Get the joint angles from the simulation
        rospy.Subscriber('/dvrk_psm1/joint_states', JointState, add_psm1_jnt)
        rospy.Subscriber('/dvrk_psm2/joint_states', JointState, add_psm2_jnt)
        
    # Get the joint angles from MTM hardware
    rospy.Subscriber('/dvrk/MTML/position_joint_current', JointState, mtml_cb)
#     rospy.Subscriber('/dvrk/MTMR/position_joint_current', JointState, add_psm1_jnt)
    
    # Detect whether or not the camera clutch is being pressed
    rospy.Subscriber('/dvrk/footpedals/camera', Bool, camera_clutch_cb)
    
    # Move the hardware from the simulation
#     rospy.Subscriber('/dvrk_psm1/joint_states', JointState, move_psm1)
#     rospy.Subscriber('/dvrk_psm2/joint_states', JointState, move_psm2)
    
    # Subscribe to fakecam images
    rospy.Subscriber('/fakecam_node/fake_image_left', Image, left_image_cb)
    rospy.Subscriber('/fakecam_node/fake_image_right', Image, right_image_cb)
    
    # Publish images
    image_left_pub = rospy.Publisher('autocamera_image_left', Image, queue_size=10)
    image_right_pub = rospy.Publisher('autocamera_image_right', Image, queue_size=10)
    
    rospy.spin()

if __name__ == "__main__":
    main()