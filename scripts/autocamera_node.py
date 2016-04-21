#!/usr/bin/env python

import sys
import rospy
from autocamera_algorithm import Autocamera
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
from rospy.core import logerror

class Autocamera_node_handler:
    # move the actual ecm with sliders?
    __MOVE_ECM_WITH_SLIDERS__ = False
    class MODE:
        simulation = "SIMULATION"
        hardware = "HARDWARE"
        sliders = "SLIDERS"
    
    
    
    DEBUG = True
    
    def __init__(self):
        self.__AUTOCAMERA_MODE__ = self.MODE.simulation
        
        self.autocamera = Autocamera() # Instantiate the Autocamera Class
        
        self.jnt_msg = JointState()
        self.joint_angles = {'ecm':None, 'psm1':None, 'psm2':None}
        self.cam_info = {'left':CameraInfo(), 'right':CameraInfo()}
        
        self.last_ecm_jnt_pos = None
        
        self.first_run = True
        
        self.ecm_robot=None
        self.psm1_robot=None
        
        self.mtmr_robot=None
        self.mtmr_kin=None
        
        self.mtml_robot=None
        self.mtml_kin=None
        
        self.camera_clutch_pressed = False
        
        self.ecm_manual_control_lock_mtml_msg = None
        self.ecm_manual_control_lock_ecm_msg = None
        
        self.ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        self.ecm_kin = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[-1].name)
        self.psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        self.psm1_kin = KDLKinematics(self.psm1_robot, self.psm1_robot.links[0].name, self.psm1_robot.links[-1].name)
        self.mtmr_robot = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
        self.mtmr_kin = KDLKinematics(self.mtmr_robot, self.mtmr_robot.links[0].name, self.mtmr_robot.links[-1].name)
        self.mtml_robot = URDF.from_parameter_server('/dvrk_mtml/robot_description')
        self.mtml_kin = KDLKinematics(self.mtml_robot, self.mtml_robot.links[0].name, self.mtml_robot.links[-1].name)
            
        self.mtml_start_position = None
        self.mtml_end_position = None
        self.initialize_psms_initialized = 30
        
        self.__init_nodes__() # initialize all the nodes, subscribers and publishers
        
    def __init_nodes__(self):
        self.ecm_hw = robot('ECM')
        self.psm1_hw = robot('PSM1')
        self.psm2_hw = robot('PSM2')
        
        rospy.init_node('autocamera_node')
        
        # Publishers to the simulation
        self.ecm_pub = rospy.Publisher('autocamera_node', JointState, queue_size=10)
        self.psm1_pub = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=10)
        self.psm2_pub = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=10)
        
        # Get the joint angles from the simulation
        rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.add_ecm_jnt)
        rospy.Subscriber('/fakecam_node/camera_info', CameraInfo, self.get_cam_info)
        
        if self.__AUTOCAMERA_MODE__ == "HARDWARE" :
            # Get the joint angles from the hardware and move the simulation from hardware
            rospy.Subscriber('/dvrk/PSM1/position_joint_current', JointState, self.add_psm1_jnt)
            rospy.Subscriber('/dvrk/PSM2/position_joint_current', JointState, self.add_psm2_jnt)
        elif self.__AUTOCAMERA_MODE__ == "SIMULATION":
            # Get the joint angles from the simulation
            rospy.Subscriber('/dvrk_psm1/joint_states', JointState, self.add_psm1_jnt)
            rospy.Subscriber('/dvrk_psm2/joint_states', JointState, self.add_psm2_jnt)
            
        # Get the joint angles from MTM hardware
        rospy.Subscriber('/dvrk/MTML/position_joint_current', JointState, self.mtml_cb)
    #     rospy.Subscriber('/dvrk/MTMR/position_joint_current', JointState, add_psm1_jnt)
        
        # Detect whether or not the camera clutch is being pressed
        rospy.Subscriber('/dvrk/footpedals/camera', Bool, self.camera_clutch_cb)
        
        # Move the hardware from the simulation
    #     rospy.Subscriber('/dvrk_psm1/joint_states', JointState, self.move_psm1)
    #     rospy.Subscriber('/dvrk_psm2/joint_states', JointState, self.move_psm2)
        
        # Subscribe to fakecam images
        rospy.Subscriber('/fakecam_node/fake_image_left', Image, self.left_image_cb)
        rospy.Subscriber('/fakecam_node/fake_image_right', Image, self.right_image_cb)
    #     
        # Publish images
        self.image_left_pub = rospy.Publisher('autocamera_image_left', Image, queue_size=10)
        self.image_right_pub = rospy.Publisher('autocamera_image_right', Image, queue_size=10)

    def spin(self):
        rospy.spin()
                
    def logerror(self, msg):
        if self.DEBUG:
            rospy.logerr(msg)
            
    def ecm_manual_control_lock(self, msg, fun):
        if fun == 'ecm':
            self.ecm_manual_control_lock_ecm_msg = msg
        elif fun == 'mtml':
            self.ecm_manual_control_lock_mtml_msg = msg
        
        if self.ecm_manual_control_lock_mtml_msg != None and self.ecm_manual_control_lock_ecm_msg != None:
            self.ecm_manual_control(self.ecm_manual_control_lock_mtml_msg, self.ecm_manual_control_lock_ecm_msg)
            self.ecm_manual_control_lock_mtml_msg = None
            self.ecm_manual_control_lock_ecm_msg = None
    
    
    def ecm_manual_control(self, mtml_msg, ecm_msg):
        # TODO: Find forward kinematics from mtmr and ecm. Move mtmr. Find the movement vector. Add it to the 
        # ecm position, use inverse kinematics and move the ecm.
            
        self.logerror("mtml" + self.mtml_kin.get_joint_names().__str__())
        start_coordinates,_ = self.mtml_kin.FK( self.mtml_start_position.position[:-1]) # Returns (position, rotation)
        end_coordinates,_ = self.mtml_kin.FK(mtml_msg.position[:-1])
        
        self.logerror(start_coordinates.__str__())
        self.logerror(end_coordinates.__str__())
        
        diff = np.subtract(end_coordinates, start_coordinates)
        self.logerror("diff = " + diff.__str__())
        
        # Find the ecm 3d coordinates, add the 'diff' to it, then do an inverse kinematics
        ecm_coordinates,_ = self.ecm_kin.FK(ecm_msg.position[0:2] + ecm_msg.position[-2:]) # There are a lot of excessive things here that we don't need
        ecm_pose = self.ecm_kin.forward(ecm_msg.position[0:2] + ecm_msg.position[-2:])
        
        # Figure out the new orientation and position to be used in the inverse kinematics
        b,_ = self.ecm_kin.FK([ecm_msg.position[0],ecm_msg.position[1],.14,ecm_msg.position[3]])
        keyhole, _ = self.ecm_kin.FK([0,0,0,0])
        ecm_current_direction = b-keyhole
        new_ecm_coordinates = np.add(ecm_coordinates, diff)
        ecm_new_direction = new_ecm_coordinates - keyhole
        r = self.autocamera.find_rotation_matrix_between_two_vectors(ecm_current_direction, ecm_new_direction)
        
        ecm_pose[0:3,0:3] =  r* ecm_pose[0:3,0:3] 
        ecm_pose[0:3,3] = new_ecm_coordinates
        new_ecm_joint_angles = self.ecm_kin.inverse(ecm_pose)
        
        new_ecm_msg = ecm_msg; new_ecm_msg.position = new_ecm_joint_angles; new_ecm_msg.name = self.ecm_kin.get_joint_names()
        
        self.logerror("ecm_new_direction " + ecm_new_direction.__str__())
        self.logerror('ecm_coordinates' + ecm_coordinates.__str__())
        self.logerror("ecm_pose " + ecm_pose.__str__())
        self.logerror("new_ecm_joint_angles " + new_ecm_joint_angles.__str__())
        self.logerror("new_ecm_msg" + new_ecm_msg.__str__())
        
        self.ecm_pub.publish(new_ecm_msg)
        
        
    def camera_clutch_cb(self, msg):
        self.camera_clutch_pressed = msg.data
        self.logerror('Camera Clutch : ' + self.camera_clutch_pressed.__str__())
        
    
    def mtml_cb(self, msg):
        if self.camera_clutch_pressed :
            if self.mtml_start_position == None:
                self.mtml_start_position = msg
            self.mtml_end_position = msg
            
            # move the ecm based on the movement of MTML
            self.ecm_manual_control_lock(msg, 'mtml')
        else:
            self.mtml_start_position = msg
            self.mtml_end_position = None
    
    def move_psm1(self, msg):
        jaw = msg.position[-3]
        msg = self.autocamera.extract_positions(msg, 'psm1')
        
        msg.name = msg.name + ['jaw']
        msg.position = msg.position + [jaw]
        
        self.psm1_hw.move_joint_list(msg.position, interpolate=True)
    
    def move_psm2(self, msg):
        time.sleep(.5)
        jaw = msg.position[-3]
        msg = self.autocamera.extract_positions(msg, 'psm2')
        
        msg.name = msg.name + ['jaw']
        msg.position = msg.position + [jaw]
        
        self.psm2_hw.move_joint_list(msg.position, interpolate=first_run)
    
    # ecm callback    
    def add_ecm_jnt(self, msg):
        if self.camera_clutch_pressed == False and msg != None:
            if self.__MOVE_ECM_WITH_SLIDERS__ == False:
                self.add_jnt('ecm', msg)
            else:
                temp = list(msg.position[:2]+msg.position[-2:])
                r = self.ecm_hw.move_joint_list(temp, interpolate=first_run)
                if r == True:
                    self.first_run = False
        else:
            self.ecm_manual_control_lock(msg, 'ecm')
            
    # psm1 callback    
    def add_psm1_jnt(self, msg):
        if self.camera_clutch_pressed == False and msg != None:
            # We need to set the names, otherwise the simulation won't move
            if self.__AUTOCAMERA_MODE__ == "HARDWARE" :
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.psm1_pub.publish(msg)
            self.add_jnt('psm1', msg)
                
         
    # psm2 callback    
    def add_psm2_jnt(self, msg):
        if self.camera_clutch_pressed == False and msg != None:
            if self.__AUTOCAMERA_MODE__ == "HARDWARE" :
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.psm2_pub.publish(msg)
            self.add_jnt('psm2', msg)
                
    def add_jnt(self, name, msg):
        self.joint_angles[name] = msg
        
        if not None in self.joint_angles.values():
            
            if self.initialize_psms_initialized>0:        
                self.initialize_psms()
                time.sleep(.01)
                self.initialize_psms_initialized -= 1
            
            try:
                jnt_msg = 'error'
                jnt_msg = self.autocamera.compute_viewangle(self.joint_angles, self.cam_info)
                t = time.time()
                
                self.ecm_pub.publish(jnt_msg)
                jnt_msg.position = [ round(i,4) for i in jnt_msg.position]
                if len(jnt_msg.position) != 4 or len(jnt_msg.name) != 4 :
                    return
                
                if self.__AUTOCAMERA_MODE__ == "HARDWARE":
                    pos = jnt_msg.position
                    result = self.ecm_hw.move_joint_list(pos, index=[0,1,2,3], interpolate=self.first_run)
                     
                    # Interpolate the insertion joint individually and the rest without interpolation
                    pos = [jnt_msg.position[2]]
                    
                    if result:
                        self.first_run = False
    #              
                
    #                 
            except TypeError:
    #             rospy.logerr('Exception : ' + TypeError.message.__str__())
                pass
                
            self.joint_angles = dict.fromkeys(self.joint_angles, None)    
        
    
    # camera info callback
    def get_cam_info(self, msg):
        if msg.header.frame_id == '/fake_cam_left_optical_link':
            self.cam_info['left'] = msg
        elif msg.header.frame_id == '/fake_cam_right_optical_link':
            self.cam_info['right'] = msg
        
    
    def image_cb(self, image_msg, camera_name):
        image_pub = {'left':self.image_left_pub, 'right':self.image_right_pub}[camera_name]
        
        bridge = cv_bridge.CvBridge()
        
        im = bridge.imgmsg_to_cv2(image_msg, 'rgb8')
        
        tool1_name = {'left':'l1', 'right':'r1'}
        tool2_name = {'left':'l2', 'right':'r2'}
        toolm_name = {'left':'lm', 'right':'rm'}
        
        tool1 = self.autocamera.zoom_level_positions[tool1_name[camera_name]]; tool1 = tuple(int(i) for i in tool1)
        tool2 = self.autocamera.zoom_level_positions[tool2_name[camera_name]]; tool2 = tuple(int(i) for i in tool2)
        toolm = self.autocamera.zoom_level_positions[toolm_name[camera_name]]; toolm = tuple(int(i) for i in toolm)
        
        rotate_180 = lambda  p : (640-p[0], 480-p[2])
    
        cv2.circle(im, tool1, 10, (0,255,0), -1)
        cv2.circle(im, tool2, 10, (0,255,255), -1)
        cv2.circle(im, toolm, 10, (0,0,255), -1)
        
        cv2.circle(im, (0,0), 20, (255,0,0), -1)
        cv2.circle(im, (640,480), 20, (255,0,255), -1)
        
        new_image = bridge.cv2_to_imgmsg(im, 'rgb8')
    
        new_image.header.seq = image_msg.header.seq
        new_image.header.stamp = image_msg.header.stamp
        new_image.header.frame_id = image_msg.header.frame_id
        
        image_pub.publish(new_image)
        
    def left_image_cb(self, image_msg):
        self.image_cb(image_msg, 'left')    
        
    def right_image_cb(self, image_msg):
        self.image_cb(image_msg, 'right')
    
    
    def initialize_psms(self):
        if self.__AUTOCAMERA_MODE__ == "SIMULATION" : 
            msg = JointState()
            msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
            msg.position = [0.84 , -0.65, 0.10, 0.00, 0.00, 0.00, 0.00]
            self.psm1_pub.publish(msg)
            msg.position = [-0.84 , -0.53, 0.10, 0.00, 0.00, 0.00, 0.00]
            self.psm2_pub.publish(msg)
            rospy.logerr('psms initialized')
    
    def set_mode(self, mode):
        """ Values:
            MODE.simulation
            MODE.hardware
        """
        self.__AUTOCAMERA_MODE__ = mode
        

def main():
    node_handler = Autocamera_node_handler()
    node_handler.set_mode(node_handler.MODE.simulation)
    node_handler.spin()

if __name__ == "__main__":
    main()