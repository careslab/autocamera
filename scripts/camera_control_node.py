#!/usr/bin/env python

import sys
import rospy
from autocamera_algorithm import Autocamera
import os
import tf
import numpy as np
import cv2
import cv_bridge
from sensor_msgs import msg 
from arm import arm as robot
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from types import NoneType
from mtm import mtm 
import sensor_msgs
from sensor_msgs.msg import Joy
from geometry_msgs.msg._PoseStamped import PoseStamped
import std_msgs
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from Crypto.Signature.PKCS1_PSS import PSS_SigScheme
import sensor_msgs
import time
from PyQt4.QtGui import *
from PyQt4.QtCore import pyqtSlot
import threading
from PyQt4.QtCore import QThread
from std_msgs.msg._Empty import Empty
from geometry_msgs.msg import PoseStamped, Pose
from hrl_geom import pose_converter

class camera_handler:
    """
        This is a base class for all classes that will manipulate the camera movements
    """
    class MODE:
        """
            simulation mode: Use the hardware for the master side, 
                            and simulation for the ECM
            hardware mode: Use hardware for both the master side,
                            and the ECM
        """
        simulation = "SIMULATION"
        hardware = "HARDWARE"

    
    def shutdown(self):
        rospy.signal_shutdown('shutting down ' + self.__name__)
        
    def set_mode(self, mode):
        """ Values:
            MODE.simulation
            MODE.hardware
        """
        self.__clutchNGo_mode__ = mode
        
    def spin(self):
        rospy.spin()
        
class Autocamera_node_handler:
    # move the actual ecm with sliders?
    __MOVE_ECM_WITH_SLIDERS__ = False
    class MODE:
        simulation = "SIMULATION"
        hardware = "HARDWARE"
        sliders = "SLIDERS"    
    
    DEBUG = True
    
    def __init__(self):
        self.t = time.time()
        
        self.__AUTOCAMERA_MODE__ = self.MODE.simulation
        
        self.autocamera = Autocamera() # Instantiate the Autocamera Class
        
        self.jnt_msg = JointState()
        self.joint_angles = {'ecm':None, 'psm1':None, 'psm2':None}
        self.cam_info = {'left':CameraInfo(), 'right':CameraInfo()}
        
        self.last_ecm_jnt_pos = None
        
        self.first_run = True
        
        # For forward and inverse kinematics
        self.ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        self.ecm_kin = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[-1].name)
        self.psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        self.psm1_kin = KDLKinematics(self.psm1_robot, self.psm1_robot.links[0].name, self.psm1_robot.links[-1].name)
        self.mtml_robot = URDF.from_parameter_server('/dvrk_mtml/robot_description')
        self.mtml_kin = KDLKinematics(self.mtml_robot, self.mtml_robot.links[0].name, self.mtml_robot.links[-1].name)
        self.mtmr_robot = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
        self.mtmr_kin = KDLKinematics(self.mtmr_robot, self.mtmr_robot.links[0].name, self.mtmr_robot.links[-1].name)
        
        
        # For camera clutch control    
        self.camera_clutch_pressed = False        
        self.ecm_manual_control_lock_mtml_msg = None
        self.ecm_manual_control_lock_ecm_msg = None
        self.mtml_start_position = None
        self.mtml_end_position = None
        
        self.initialize_psms_initialized = 30
        self.__DEBUG_GRAPHICS__ = False
        
        
    def __init_nodes__(self):
        self.ecm_hw = robot('ECM')
        self.psm1_hw = robot('PSM1')
        self.psm2_hw = robot('PSM2')
            
        #rospy.init_node('autocamera_node')
        
        self.logerror("start", debug=True)
        
        # Publishers to the simulation
        self.ecm_pub = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=10)
        self.psm1_pub = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=10)
        self.psm2_pub = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=10)
        
        # Get the joint angles from the simulation
        self.sub_ecm_sim = rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.add_ecm_jnt)
        self.sub_caminfo = rospy.Subscriber('/fakecam_node/camera_info', CameraInfo, self.get_cam_info)
        
        try:
            self.sub_psm1_sim.unregister()
            self.sub_psm2_sim.unregister()
            self.sub_psm1_hw.unregister()
            self.sub_psm2_hw.unregister()
        except Exception:
            pass
        if self.__AUTOCAMERA_MODE__ == self.MODE.hardware :
            # Get the joint angles from the hardware and move the simulation from hardware
            self.sub_psm1_hw = rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, self.add_psm1_jnt)
            self.sub_psm2_hw = rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, self.add_psm2_jnt)
            
            
        elif self.__AUTOCAMERA_MODE__ == self.MODE.simulation:
            # Get the joint angles from the simulation
            self.sub_psm1_sim = rospy.Subscriber('/dvrk_psm1/joint_states', JointState, self.add_psm1_jnt)
            self.sub_psm2_sim = rospy.Subscriber('/dvrk_psm2/joint_states', JointState, self.add_psm2_jnt)
            
            # If hardware is connected, subscribe to it and set the psm joint angles in the simulation from the hardware
            self.sub_psm1_hw = rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, self.add_psm1_jnt_from_hw)
            self.sub_psm2_hw = rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, self.add_psm2_jnt_from_hw)
            
            
        # Get the joint angles from MTM hardware
        ##rospy.Subscriber('/dvrk/MTML/position_joint_current', JointState, self.mtml_cb)
    #     rospy.Subscriber('/dvrk/MTMR/position_joint_current', JointState, add_psm1_jnt)
        
        # Detect whether or not the camera clutch is being pressed
##         rospy.Subscriber('/dvrk/footpedals/camera', Bool, self.camera_clutch_cb)
        
        # Move the hardware from the simulation
    #     rospy.Subscriber('/dvrk_psm1/joint_states', JointState, self.move_psm1)
    #     rospy.Subscriber('/dvrk_psm2/joint_states', JointState, self.move_psm2)
        
        if self.__DEBUG_GRAPHICS__ == True:
            # Subscribe to fakecam images
            self.sub_fake_image_left = rospy.Subscriber('/fakecam_node/fake_image_left', Image, self.left_image_cb)
            self.sub_fake_image_right = rospy.Subscriber('/fakecam_node/fake_image_right', Image, self.right_image_cb)
         
        # Publish images
        self.image_left_pub = rospy.Publisher('autocamera_image_left', Image, queue_size=10)
        self.image_right_pub = rospy.Publisher('autocamera_image_right', Image, queue_size=10)

    def shutdown(self):
        try:
            if self.__DEBUG_GRAPHICS__ == True:
                self.sub_fake_image_left.unregister()
                self.sub_fake_image_right.unregister()
                
            if self.__AUTOCAMERA_MODE__ == self.MODE.simulation:
                self.sub_psm1_sim.unregister()
                self.sub_psm2_sim.unregister()
                self.sub_ecm_sim.unregister()
            self.image_left_pub.unregister()
            self.image_right_pub.unregister()
            self.sub_psm1_hw.unregister()
            self.sub_psm2_hw.unregister()
            self.ecm_pub.unregister()
            self.sub_caminfo.unregister()
            self.psm1_pub.unregister()
            self.psm2_pub.unregister()
            
            print( "Shutting down " + self.__class__.__name__)
            
        except Exception:
            print("couldn't unregister all the topics")
#         rospy.signal_shutdown('shutting down Autocamera')
        
    # This needs to be run before anything can be expected
    def spin(self):
        self.__init_nodes__() # initialize all the nodes, subscribers and publishers
        rospy.spin()
         
    def debug_graphics(self, has_graphics):
        self.__DEBUG_GRAPHICS__ = has_graphics
            
    def logerror(self, msg, debug = False):
        if self.DEBUG or debug:
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
            
            #Freeze mtms
            ### CODE HERE ###
            
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
       
    
    def add_psm1_jnt_from_hw(self, msg):
        if self.camera_clutch_pressed == False and msg != None:
            if self.__AUTOCAMERA_MODE__ == self.MODE.simulation:
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.psm1_pub.publish(msg)

    def add_psm2_jnt_from_hw(self, msg):
        if self.camera_clutch_pressed == False and msg != None:
            if self.__AUTOCAMERA_MODE__ == self.MODE.simulation:
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.psm2_pub.publish(msg)
                
    # psm1 callback    
    def add_psm1_jnt(self, msg):
        if self.camera_clutch_pressed == False and msg != None:
            # We need to set the names, otherwise the simulation won't move
            if self.__AUTOCAMERA_MODE__ == self.MODE.hardware:
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.psm1_pub.publish(msg)
            self.add_jnt('psm1', msg)
                
         
    # psm2 callback    
    def add_psm2_jnt(self, msg):
        if self.camera_clutch_pressed == False and msg != None:
            if self.__AUTOCAMERA_MODE__ == self.MODE.hardware :
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.psm2_pub.publish(msg)
            self.add_jnt('psm2', msg)
                
    def add_jnt(self, name, msg):
        self.joint_angles[name] = msg
        
        if not None in self.joint_angles.values():
            if self.initialize_psms_initialized>0 and self.__AUTOCAMERA_MODE__ == self.MODE.simulation:        
                self.initialize_psms()
                time.sleep(.01)
                self.initialize_psms_initialized -= 1
            try:
                jnt_msg = 'error'
                jnt_msg = self.autocamera.compute_viewangle(self.joint_angles, self.cam_info)
                
                self.ecm_pub.publish(jnt_msg)
                
                jnt_msg.position = [ round(i,4) for i in jnt_msg.position]
                if len(jnt_msg.position) != 4 or len(jnt_msg.name) != 4 :
                    return
                #return # stop here until we co-register the arms
                if self.__AUTOCAMERA_MODE__ == self.MODE.hardware:
                    pos = jnt_msg.position
                    result = self.ecm_hw.move_joint_list(pos, index=[0,1,2,3], interpolate=self.first_run)
                    # Interpolate the insertion joint individually and the rest without interpolation
 #                   pos = [jnt_msg.position[2]]
#                    result = self.ecm_hw.move_joint_list(pos, index=[2], interpolate=True)
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
        
        if not None in self.autocamera.zoom_level_positions:
            tool1 = self.autocamera.zoom_level_positions[tool1_name[camera_name]]; tool1 = tuple(int(i) for i in tool1)
            tool2 = self.autocamera.zoom_level_positions[tool2_name[camera_name]]; tool2 = tuple(int(i) for i in tool2)
            toolm = self.autocamera.zoom_level_positions[toolm_name[camera_name]]; toolm = tuple(int(i) for i in toolm)
            
            rotate_180 = lambda  p : (640-p[0], 480-p[2])
            w = 640 ; h = 480;
#             outer_margin = .7; inner_margin = .2;
#             ozone = [[int(outer_margin * w), int(outer_margin * h)],
#                      [int( (1-outer_margin) * w), int(outer_margin * h)],
#                      [int((1-outer_margin) * w), int((1-outer_margin) * h)],
#                      [int(outer_margin * w), int((1-outer_margin) * h)]]
#             # Draw the outer margin
#             for i in range(-1, len(ozone)-1):
#                 first = i
#                 second = i+1
#                 if i == -1: first = len(ozone) - 1
#                 cv2.line(im, tuple(ozone[first]),tuple(ozone[second]), (255,0,0))
#                 
#             izone = [[int(inner_margin * w), int(inner_margin * h)],
#                      [int( (1-inner_margin) * w), int(inner_margin * h)],
#                      [int((1-inner_margin) * w), int((1-inner_margin) * h)],
#                      [int(inner_margin * w), int((1-inner_margin) * h)]]
#             # Draw the outer margin
#             for i in range(-1, len(izone)-1):
#                 first = i
#                 second = i+1
#                 if i == -1: first = len(izone) - 1
#                 cv2.line(im, tuple(izone[first]),tuple(izone[second]), (0,0,255))
            
            inner_radius = 0.1 ; deadzone_radius = 0.2 + inner_radius;
            mid_point = ( int(tool1[0]+tool2[0])/2, int(tool1[1] + tool2[1])/2)  

            cv2.circle(im, mid_point, int(deadzone_radius * w), (128,128,128))
            cv2.circle(im, mid_point, int(inner_radius * w), (255,255,255))    
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
        if self.__AUTOCAMERA_MODE__ == self.MODE.simulation : 
            msg = JointState()
            msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
            msg.position = [0.84 , -0.65, 0.10, 0.00, 0.00, 0.00, 0.00]
            self.psm1_pub.publish(msg)
            msg.position = [-0.84 , -0.53, 0.10, 0.00, 0.00, 0.00, 0.00]
            self.psm2_pub.publish(msg)
            self.logerror('psms initialized!')
            
    
    def set_mode(self, mode):
        """ Values:
            MODE.simulation
            MODE.hardware
        """
        self.__AUTOCAMERA_MODE__ = mode
        
        

class ClutchControl:
    # General idea:
    # Clutch
    # Get mtmr forward kinematics
    # Move mtmr
    # Get forward kinematics again
    # get the vector that was traveled
    # map x,y,z to pan, tilt, insertion in the camera
    # When unclutched don't move the camera anymore
    
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
        self.camera_clutch_pressed = False
        self.movement_scale = 1 
        self.joint_angles = [0,0,0,0]
        self.center = [0,0,0,0]
        
        self.flag = True
        
        self.mtml_pos = [0,0,0]
        self.mtml_joint_angles = [0,0,0,0,0,0,0]
        
        self.mtmr_pos = [0,0,0]
        self.mtmr_joint_angles = [0,0,0,0,0,0,0]
        
        self.mtmr_starting_point =  [0,0,0,0,0,0,0]
        
        self.__init_nodes__()
        
    def __init_nodes__(self):
#         rospy.init_node('ecm_clutch_control')
        
        self.ecm_hw = robot("ECM")
        self.mtmr_hw = robot("MTMR")
        self.mtml_hw = robot("MTML")
        
        self.mtml_hw_pub = rospy.Publisher('/dvrk/MTML/set_position_joint', JointState, queue_size=1)
        self.mtmr_hw_pub = rospy.Publisher('/dvrk/MTMR/set_position_joint', JointState, queue_size=1)
        
        self.ecm_sim = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=10)
        self.ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        self.mtmr_robot = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
        self.mtml_robot = URDF.from_parameter_server('/dvrk_mtml/robot_description')
        self.psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        self.psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
        
        self.ecm_kin = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[-1].name)
        self.mtmr_kin = KDLKinematics(self.mtmr_robot, self.mtmr_robot.links[0].name, self.mtmr_robot.links[-1].name)
        self.mtml_kin = KDLKinematics(self.mtml_robot, self.mtml_robot.links[0].name, self.mtml_robot.links[-1].name)
        self.psm1_kin = KDLKinematics(self.psm1_robot, self.psm1_robot.links[0].name, self.psm1_robot.links[-1].name)
        self.psm2_kin = KDLKinematics(self.psm2_robot, self.psm2_robot.links[0].name, self.psm2_robot.links[-1].name)
        
        self.mtml_orientation = mtm('MTML')
        self.mtmr_orientation = mtm('MTMR')
        
        self.mtml_psm2_orientation = rospy.Publisher('/dvrk/MTML_PSM2/lock_rotation', Bool, queue_size=1, latch=True )
        self.mtml_psm2_translation = rospy.Publisher('/dvrk/MTML_PSM2/lock_translation', Bool, queue_size=1, latch=True )
        
        self.mtmr_psm1_orientation = rospy.Publisher('/dvrk/MTMR_PSM1/lock_rotation', Bool, queue_size=1, latch=True )
        self.mtmr_psm1_translation = rospy.Publisher('/dvrk/MTMR_PSM1/lock_translation', Bool, queue_size=1, latch=True )
        
        self.mtmr_psm1_teleop = rospy.Publisher('/dvrk/MTMR_PSM1/set_desired_state', String, latch=True, queue_size=1)
        self.mtml_psm2_teleop = rospy.Publisher('/dvrk/MTML_PSM2/set_desired_state', String, latch=True, queue_size=1)
        
        self.sub_ecm_cb = None
        if self.__mode__ == self.MODE.simulation:
            self.sub_ecm_cb = rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.ecm_cb)
        elif self.__mode__ == self.MODE.hardware:
            self.sub_ecm_cb = rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.ecm_cb)
#             self.ecm_hw.home()
            self.ecm_hw.move_joint_list([0.0,0.0,0.0,0.0], interpolate=True)
        
        self.camera_clutch_pressed = False
        self.head_sensor_pressed = False
        self.sub_camera_clutch_cb = rospy.Subscriber('/dvrk/footpedals/camera_minus', Joy, self.camera_clutch_cb )
        self.sub_headsensor_cb = rospy.Subscriber('/dvrk/footpedals/coag', Joy, self.camera_headsensor_cb )
        self.mtml_starting_point = None
        
        self.sub_mtml_cart_cb = rospy.Subscriber('/dvrk/MTML/position_cartesian_local_current', PoseStamped, self.mtml_cb)
        self.sub_mtml_joint_cb = rospy.Subscriber('/dvrk/MTML/state_joint_current', JointState, self.mtml_joint_angles_cb)
        
        self.sub_mtmr_cart_cb = rospy.Subscriber('/dvrk/MTMR/position_cartesian_local_current', PoseStamped, self.mtmr_cb)
        self.sub_mtmr_joint_cb = rospy.Subscriber('/dvrk/MTMR/state_joint_current', JointState, self.mtmr_joint_angles_cb)
        
        self.sub_psm1_joint_cb = rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, self.psm1_joint_angles_cb)
        self.sub_psm2_joint_cb = rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, self.psm2_joint_angles_cb)
        
#         self.mtml_orientation.lock_orientation_as_is()
#         self.mtml_orientation.unlock_orientation()
    
    def shutdown(self):
        try:
            self.sub_ecm_cb.unregister()
            self.sub_camera_clutch_cb.unregister()
            self.sub_headsensor_cb.unregister()
            self.sub_mtml_cart_cb.unregister()
            self.sub_mtml_joint_cb.unregister()
            self.sub_mtmr_cart_cb.unregister()
            self.sub_mtmr_joint_cb.unregister()
            self.sub_psm1_joint_cb.unregister()
            self.sub_psm2_joint_cb.unregister()
            
            print( "Shutting down " + self.__class__.__name__)
            
        except(e):
            print("couldn't unregister all the topics")
            pass
#         rospy.signal_shutdown('shutting down ClutchControl')
        
    def set_mode(self, mode):
        """ Values:
            MODE.simulation
            MODE.hardware
        """
        self.__mode__ = mode
            
    def spin(self):
        rospy.spin()

    def mtml_joint_angles_cb(self, msg):
        self.mtml_joint_angles = list(msg.position)
        self.move_mtm_out_of_the_way()
    
    def mtmr_joint_angles_cb(self, msg):
        self.mtmr_joint_angles = list(msg.position)
        
    def psm1_joint_angles_cb(self,msg):
        self.psm1_joint_angles = list(msg.position)
    
    def psm2_joint_angles_cb(self,msg):
        self.psm2_joint_angles = list(msg.position)
    
    def mtml_cb(self, msg):
#         msg.pose.position.x
        if self.camera_clutch_pressed:
            if type(self.mtml_starting_point) == NoneType:
                self.mtml_starting_point = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            
            # we may multiply the current_position and mtml_pos_before_clutch by some transformation matrix so
            # the hand controllers feel more intuitive
            
            current_position = self.mtml_kin.forward(list(self.mtml_joint_angles)[0:-1])[0:3,3] 
            movement_vector = current_position-self.mtml_pos_before_clutch
            self.move_mtm_centerpoints()
            print("movement_vector = {}, {}, {}".format(movement_vector[0], movement_vector[1], movement_vector[2]))
            self.mtml_pos = current_position
            self.ecm_pan_tilt(movement_vector)
        else:
#             self.center = self.joint_angles
            try:
                self.mtml_pos_before_clutch = self.mtml_kin.forward(list(self.mtml_joint_angles)[0:-1])[0:3,3]
            except:
                pass
    
    def mtmr_cb(self, msg):
        pass
        if self.camera_clutch_pressed:
            pass
            if type(self.mtmr_starting_point) == NoneType:
                self.mtmr_starting_point = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            current_position = self.mtmr_kin.forward(list(self.mtmr_joint_angles)[0:-1])[0:3,3] 
            movement_vector = current_position-self.mtmr_starting_point
            self.mtmr_pos = current_position
#             self.ecm_pan_tilt(movement_vector[0:2])
        else:
#             self.center = self.joint_angles
            try :
                self.mtmr_pos_before_clutch = self.mtmr_kin.forward(list(self.mtmr_joint_angles)[0:-1])[0:3,3]
            except:
                pass
    
    def move_mtm_out_of_the_way(self):
        if self.head_sensor_pressed == True and self.camera_clutch_pressed == False:
            if self.mtml_hw.get_robot_state() == 'DVRK_POSITION_GOAL_CARTESIAN' and self.flag == True:
                self.flag == False
                ml = self.mtml_joint_angles[3]
                mr = self.mtml_joint_angles[3]
                delta = .1
                
                mtml_joints = JointState()
                mtml_joints.name = ['outer_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_platform', 'wrist_pitch', 'wrist_yaw', 'wrist_roll', 'finger_grips']
                mtml_joints.position = self.mtml_joint_angles
                mtml_joints.position[3] += (-1.58 -mtml_joints.position[3])/10.0  
                mtml_joints.effort = [0, 0, 0, delta, 0, 0, 0, 0]
                print("mtml_joints = " + mtml_joints.__str__())
                self.mtml_hw_pub.publish(mtml_joints)
                
                mtmr_joints = JointState()
                mtmr_joints.name = ['outer_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_platform', 'wrist_pitch', 'wrist_yaw', 'wrist_roll', 'finger_grips']
                mtmr_joints.position = self.mtmr_joint_angles
                new_p = list(mtmr_joints.position)
                new_p[3] = 1.58
                mtmr_joints.position[3] +=  (1.58 - mtmr_joints.position[3])/10.0 
                mtmr_joints.effort = [0, 0, 0, delta, 0, 0, 0, 0]
                self.mtmr_hw_pub.publish(mtmr_joints)
                
    # To be completed
    def move_mtm_centerpoints(self):
        left = self.mtml_pos_before_clutch
        right = self.mtmr_pos_before_clutch
        
        mtml_pos = self.mtml_kin.forward(list(self.mtml_joint_angles)[0:-1])[0:3,3]
        mtmr_pos = self.mtmr_kin.forward(list(self.mtmr_joint_angles)[0:-1])[0:3,3]
        
        mid = (left+right)/2.0
        
        if type(mtml_pos) != NoneType:
            ml_vector = mid-left
            mr_vector = right-mid
            
            new_mid = mtml_pos + ml_vector
            new_mtmr_position = new_mid + mr_vector
            
            mtmr_pose = self.mtmr_kin.forward(list(self.mtmr_joint_angles)[0:-1])
            mtmr_pose[0:3, 3] = new_mtmr_position.reshape(3,1)
            mtmr_joint_angles = self.mtmr_kin.inverse(mtmr_pose)
#             print('mtmr_joint_angles = ' + mtmr_joint_angles.__str__())
            
            mtmr_joint_angles = [float(i) for i in mtmr_joint_angles]
            mtmr_joint_angles.append(0.0)
            self.mtmr_hw.move_joint_list(mtmr_joint_angles[0:3], [0,1,2], interpolate=False)
            
#             self.move_mtm_out_of_the_way()
             
        
        
    def enable_teleop(self):
        self.mtmr_psm1_teleop.publish(String("ENABLED"))
        self.mtml_psm2_teleop.publish(String("ENABLED"))
        
        
    def disable_teleop(self):
        self.mtml_orientation.lock_orientation_as_is()
        self.mtmr_orientation.lock_orientation_as_is()
        
        self.mtmr_psm1_teleop.publish(String("DISABLED"))
        self.mtml_psm2_teleop.publish(String("DISABLED"))
                
    def camera_headsensor_cb(self, msg):
        if msg.buttons[0] == 1:
            self.head_sensor_pressed = True
            self.enable_teleop()
        else:
            self.head_sensor_pressed = False
            self.mtml_hw.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
            self.mtmr_hw.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
                        
    def camera_clutch_cb(self, msg):
        if msg.buttons[0] == 1 and self.head_sensor_pressed: # Camera clutch pressed
            self.camera_clutch_pressed = True
            self.mtml_starting_point = None
            self.disable_teleop()
            
        else: # Camera clutch not pressed anymore
            self.camera_clutch_pressed = False
#             self.center = self.joint_angles
            if self.head_sensor_pressed:
                self.enable_teleop()
                            
    def ecm_cb(self, msg):
        if self.__mode__ == self.MODE.simulation:
            self.joint_angles = msg.position[0:2] + msg.position[-2:]
        elif self.__mode__ == self.MODE.hardware:
            self.joint_angles = msg.position[0:3] + tuple([0])
            
        if self.camera_clutch_pressed == False:
            if self.__mode__ == self.MODE.simulation:
                self.center = msg.position[0:2] + msg.position[-2:]
            elif self.__mode__ == self.MODE.hardware:
                self.center = msg.position[0:3] + tuple([0])
            self.center_cart = np.array(self.ecm_kin.FK(self.center)[0])
            
    
    def ecm_pan_tilt(self, movement_vector):
        q = []
        q = self.joint_angles
        print("ecm joint angles are : " + q.__str__())
        movement_vector = [float(i) for i in movement_vector]
        if q:
            q = list(q)
            q[0] = self.center[0] + movement_vector[0] * self.movement_scale # pitch
            q[1] = self.center[1] - movement_vector[1] * self.movement_scale # yaw
            q[2] = self.center[2] - movement_vector[2] *.2 * self.movement_scale # insertion
            
            q = [round(i,4) for i in q]
#             q = [i+j for i,j in zip(self.center, q)]
            print(q)
            self.move_ecm(q)
    
    # move ecm based on joint angles either in simulation or hardware
    def move_ecm(self, joint_angles):
        if self.__mode__ == self.MODE.simulation:
            msg = JointState()
            msg.position = joint_angles
            msg.name = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
            self.ecm_sim.publish(msg)            
        elif self.__mode__ == self.MODE.hardware:
            self.ecm_hw.move_joint_list(joint_angles, interpolate=False)
    
    def ecm_inverse(self, goal_pos):
        key_hole,_ = self.ecm_kin.FK([0,0,0,0])
        safe_angles = list(self.joint_angles)
        safe_angles[2] = 0.14
        
        ecm_pose = self.ecm_kin.forward(safe_angles)
        ab_vector = (goal_pos - key_hole)
        
        b, _ = self.ecm_kin.FK(safe_angles)
        
        ecm_current_direction = b - key_hole
        
        r = self.find_rotation_matrix_between_two_vectors(ecm_current_direction, ab_vector)
        m = np.sqrt(ab_vector[0]**2 + ab_vector[1]**2 + ab_vector[2]**2) # ab_vector's length
        
        # insertion joint length
        l = np.sqrt( (ecm_pose[0,3]-key_hole[0])**2 + (ecm_pose[1,3]-key_hole[1])**2 + (ecm_pose[2,3]-key_hole[2])**2)
        
        # Equation of the line that passes through the midpoint of the tools and the key hole
        x = lambda t: key_hole[0] + ab_vector[0] * t
        y = lambda t: key_hole[1] + ab_vector[1] * t
        z = lambda t: key_hole[2] + ab_vector[2] * t
        
        t = l/m
        
        new_ecm_position = np.array([x(t), y(t), z(t)]).reshape(3,1)
        
        ecm_pose[0:3,0:3] =  r* ecm_pose[0:3,0:3]  
        ecm_pose[0:3,3] = new_ecm_position
        
        
        new_joint_angles = self.joint_angles
        
        p = None
        try:
            p = self.ecm_kin.inverse(ecm_pose)
        except Exception as e:
            rospy.logerr('error, cannot do inverse kinematics of ecm')
        if type(p) != NoneType: 
            p[3] = 0
#             p[2] -= 0.14
            new_joint_angles = p
        else:
            from colorama import Fore, Back
            print (Fore.GREEN + Back.YELLOW+ "inverse Kinematics is failing") 
        return new_joint_angles
    
    def find_rotation_matrix_between_two_vectors(self, a,b):
        a = np.array(a).reshape(1,3)[0].tolist()
        b = np.array(b).reshape(1,3)[0].tolist()
        
        vector_orig = a / np.linalg.norm(a)
        vector_fin = b / np.linalg.norm(b)
                     
        # The rotation axis (normalised).
        axis = np.cross(vector_orig, vector_fin)
        axis_len = np.linalg.norm(axis)
        if axis_len != 0.0:
            axis = axis / axis_len
    
        # Alias the axis coordinates.
        x = axis[0]
        y = axis[1]
        z = axis[2]
        
        # The rotation angle.
        angle = np.math.acos(np.dot(vector_orig, vector_fin))
    
        # Trig functions (only need to do this maths once!).
        ca = np.math.cos(angle)
        sa = np.math.sin(angle)
        R = np.identity(3)
        # Calculate the rotation matrix elements.
        R[0,0] = 1.0 + (1.0 - ca)*(x**2 - 1.0)
        R[0,1] = -z*sa + (1.0 - ca)*x*y
        R[0,2] = y*sa + (1.0 - ca)*x*z
        R[1,0] = z*sa+(1.0 - ca)*x*y
        R[1,1] = 1.0 + (1.0 - ca)*(y**2 - 1.0)
        R[1,2] = -x*sa+(1.0 - ca)*y*z
        R[2,0] = -y*sa+(1.0 - ca)*x*z
        R[2,1] = x*sa+(1.0 - ca)*y*z
        R[2,2] = 1.0 + (1.0 - ca)*(z**2 - 1.0)
        
        R = np.matrix(R)
        return R 

class Joystick:
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
        
        self.__init_nodes__()
        
    def __init_nodes__(self):
        self.ecm_hw = robot("ECM")
        self.ecm_sim = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=10)
        self.ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        self.ecm_kin = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[-1].name)
        
        self.sub_ecm = None
        if self.__mode__ == self.MODE.simulation:
            self.sub_ecm = rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.ecm_cb)
        elif self.__mode__ == self.MODE.hardware:
            self.sub_ecm = rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.ecm_cb)
#             self.ecm_hw.home()
            self.ecm_hw.move_joint_list([0.0,0.0,0.0,0.0], interpolate=True)
            
        self.sub_joy = rospy.Subscriber('/joy', msg.Joy, self.on_joystick_change_cb)
        
    def shutdown(self):
        print('shutting down joystick control')
        try:
            self.sub_ecm.unregister()
            self.sub_joy.unregister()
            self.ecm_sim.unregister()
            
            print( "Shutting down " + self.__class__.__name__)
        except e:
            print("couldn't unregister all the topics")

    def set_mode(self, mode):
        self.__mode__ = mode
                
    def spin(self):
        self.__spin__()
    def __spin__(self):
        rospy.spin()

    def ecm_cb(self, msg):
        if self.__mode__ == self.MODE.simulation:
            self.joint_angles = msg.position[0:2] + msg.position[-2:]
        elif self.__mode__ == self.MODE.hardware:
            self.joint_angles = msg.position[0:2] + (0,0)
        if self.last_z != 0.0:
            q = list(self.center)
            q = self.ecm_zoom(self.last_z, q)
            self.move_ecm([q[2]], index = [2])
    
    def ecm_joystick_recenter(self):
        self.center[0] = self.joint_angles[0]
        self.center[1] = self.joint_angles[1]
        self.center[3] = self.joint_angles[3]
        self.last_z = 0
        self.joystick_at_zero = False     
           
    def on_joystick_change_cb(self, message):
        j = msg.Joy()
        if sum( np.abs(message.axes[0:2])) != 0 and self.joystick_at_zero == False:
            return
        self.joystick_at_zero = True
        
        if message.buttons[0] == 1:
                self.ecm_joystick_recenter()
                print('self.center = {}, self.joint_angles = {}'.format(self.center, self.joint_angles))
                
                
        elif message.axes[3] > .7 and self.joystick_at_zero == True: # The throttle 
            # Allow movement
            movement_vector = np.array(message.axes[0:2]) # left right and up down
                
            q = self.ecm_pan_tilt(movement_vector)
#             q = self.ecm_zoom(message.axes[-1], q)
            self.last_z = message.axes[-1]
            self.move_ecm([q[0], q[1],q[3]], index = [0,1,3]) 
    
    def ecm_zoom(self, z, q = []):
        print(z, q, self.joint_angles)
        if q == []:
            q = self.joint_angles
        if z == 0.0 :
            return q
        if z > 0 and q[2] < .21: # Zoom in
            q[2] = self.joint_angles[2] + .0002
        elif z < 0:
            q[2] = self.joint_angles[2] - .0002
#         if q[2] < 0 :
#             q[2] = .001
        q = [round(i,4) for i in q]
        q = [i+j for i,j in zip(self.center, q)]
#         self.joint_angles = q
        self.center[2] = q[2]
        return q
    
    def ecm_pan_tilt(self, movement_vector, q = []):
        q = []
        q = self.joint_angles
        if q:
            q = list(q)
            q[0] += .4 * movement_vector[0] * self.movement_scale
            q[1] += .08 * movement_vector[1] * self.movement_scale
            q = [round(i,4) for i in q]
            q = [i+j for i,j in zip(self.center, q)]
#             self.move_ecm(q)
            return q
    
    # move ecm based on joint angles either in simulation or hardware
    def move_ecm(self, joint_angles, index = []):
        if self.__mode__ == self.MODE.simulation:
            msg = JointState()
            msg.position = joint_angles
            msg.name = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
            self.ecm_sim.publish(msg)            
        elif self.__mode__ == self.MODE.hardware:
            if index == []:
                index = range(0,len(joint_angles))
            self.ecm_hw.move_joint_list(joint_angles, index, interpolate=False)
    
    def ecm_inverse(self, goal_pos):
        key_hole,_ = self.ecm_kin.FK([0,0,0,0])
        safe_angles = list(self.joint_angles)
        safe_angles[2] += 0.14
        
        ecm_pose = self.ecm_kin.forward(safe_angles)
        ab_vector = (goal_pos - key_hole)
        
        b, _ = self.ecm_kin.FK(safe_angles)
        
        ecm_current_direction = b - key_hole
        
        r = self.find_rotation_matrix_between_two_vectors(ecm_current_direction, ab_vector)
        m = np.sqrt(ab_vector[0]**2 + ab_vector[1]**2 + ab_vector[2]**2) # ab_vector's length
        
        # insertion joint length
        l = np.sqrt( (ecm_pose[0,3]-key_hole[0])**2 + (ecm_pose[1,3]-key_hole[1])**2 + (ecm_pose[2,3]-key_hole[2])**2)
        
        # Equation of the line that passes through the midpoint of the tools and the key hole
        x = lambda t: key_hole[0] + ab_vector[0] * t
        y = lambda t: key_hole[1] + ab_vector[1] * t
        z = lambda t: key_hole[2] + ab_vector[2] * t
        
        t = l/m
        
        new_ecm_position = np.array([x(t), y(t), z(t)]).reshape(3,1)
        
        ecm_pose[0:3,0:3] =  r* ecm_pose[0:3,0:3]  
        ecm_pose[0:3,3] = new_ecm_position
        
        
        new_joint_angles = self.joint_angles
        
        p = None
        try:
            p = self.ecm_kin.inverse(ecm_pose)
        except Exception as e:
            rospy.logerr('error')
        if p != None:  
            p[3] = 0
            p[2] -= 0.14
            new_joint_angles = p
        return new_joint_angles
    
    def find_rotation_matrix_between_two_vectors(self, a,b):
        a = np.array(a).reshape(1,3)[0].tolist()
        b = np.array(b).reshape(1,3)[0].tolist()
        
        vector_orig = a / np.linalg.norm(a)
        vector_fin = b / np.linalg.norm(b)
                     
        # The rotation axis (normalised).
        axis = np.cross(vector_orig, vector_fin)
        axis_len = np.linalg.norm(axis)
        if axis_len != 0.0:
            axis = axis / axis_len
    
        # Alias the axis coordinates.
        x = axis[0]
        y = axis[1]
        z = axis[2]
        
        # The rotation angle.
        angle = np.math.acos(np.dot(vector_orig, vector_fin))
    
        # Trig functions (only need to do this maths once!).
        ca = np.math.cos(angle)
        sa = np.math.sin(angle)
        R = np.identity(3)
        # Calculate the rotation matrix elements.
        R[0,0] = 1.0 + (1.0 - ca)*(x**2 - 1.0)
        R[0,1] = -z*sa + (1.0 - ca)*x*y
        R[0,2] = y*sa + (1.0 - ca)*x*z
        R[1,0] = z*sa+(1.0 - ca)*x*y
        R[1,1] = 1.0 + (1.0 - ca)*(y**2 - 1.0)
        R[1,2] = -x*sa+(1.0 - ca)*y*z
        R[2,0] = -y*sa+(1.0 - ca)*x*z
        R[2,1] = x*sa+(1.0 - ca)*y*z
        R[2,2] = 1.0 + (1.0 - ca)*(z**2 - 1.0)
        
        R = np.matrix(R)
        return R 
            

class camera_qt_gui:
    
    class node_name:
        clutchNGo = 'clutch_control'
        autocamera = 'Autocamera'
        joystick = 'joystick'
    class MODE:
        """
            simulation mode: Use the hardware for the master side, 
                            and simulation for the ECM
            hardware mode: Use hardware for both the master side,
                            and the ECM
        """
        simulation = "SIMULATION"
        hardware = "HARDWARE"
        
    # The following classes help create threads so the GUI would not freeze
    class thread_autocamera(QThread):
        def __init__(self, mode):
            super(QThread, self).__init__()
            self.node_handler = None
            self.__mode__ = mode
        def run(self):
            self.node_handler = Autocamera_node_handler()
            print('\nRunning {} in {}\n'.format("Autocamera",self.__mode__))
            self.node_handler.set_mode(self.__mode__)
            self.node_handler.debug_graphics(True)
            self.node_handler.spin()
        
        def kill(self):
            if self.node_handler != None:
                self.node_handler.shutdown()
                self.quit()
            
    class thread_clutchNGo(QThread):
        def __init__(self, mode):
            super(QThread, self).__init__()
            self.node_handler = None
            self.__mode__ = mode
        def run(self):
            self.node_handler = ClutchControl()
            print('\nRunning {} in {}\n'.format("Clutch and Move",self.__mode__))
            self.node_handler.set_mode(self.__mode__)
            self.node_handler.spin()
        
        def kill(self):
            if self.node_handler != None:
                self.node_handler.shutdown()
                self.quit()
                
    class thread_joystick(QThread):
        def __init__(self, mode):
            super(QThread, self).__init__()
            self.node_handler = None
            self.__mode__ = mode
        def run(self):
            self.node_handler = Joystick()
            print('\nRunning {} in {}\n'.format("Joystick Control",self.__mode__))
            self.node_handler.set_mode(self.__mode__)
            self.node_handler.spin()
        
        def kill(self):
            if self.node_handler != None:
                self.node_handler.shutdown()
                self.quit()
                
    """
        thread_home_arms:
                This thread sets the PSM base frames so the MTM's and PSM's align correctly.
                It also moves the MTM's wrists to an appropriate location
                
    """
    class thread_home_arms(QThread):
        def __init__(self):
            super(QThread, self).__init__()
            self.is_running = True
            
        def run(self):
            self.counter = 100
            psm1 = robot('PSM1')
            psm2 = robot('PSM2')
            ecm = robot('ECM')
            mtml = robot('MTML')
            mtmr = robot('MTMR')
            psm1.move_joint_list([0.0, 0.0, 0.0, 0.0], [3,4,5,6], interpolate=True)
            psm2.move_joint_list([0.0, 0.0, 0.0, 0.0], [3,4,5,6], interpolate=True)
            
            self.sub_ecm = rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.set_base_frames)
            
            
#             mtml.move_joint_list([-1.57],[3], interpolate=True)
#             mtmr.move_joint_list([1.57],[3], interpolate=True)
            
            ecm.move_joint_list([0.0,0.0,0.0,0.0], interpolate=True)
            
#             self.set_base_frames()
            
            self.is_running = False
            
        def kill(self):
            if self.is_running == False:
                self.quit()
                        
        def set_base_frames(self, msg):
            print('set_base_frames', msg.position)
            if msg.position and self.counter > 0:
                self.counter -= 1
                q = msg.position
                
                self.ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
                self.ecm_kin = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[-1].name)
                self.ecm_base = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[3].name)
                
                self.psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
                self.psm1_kin = KDLKinematics(self.psm1_robot, self.psm1_robot.links[0].name, self.psm1_robot.links[1].name)
                
                self.psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
                self.psm2_kin = KDLKinematics(self.psm2_robot, self.psm2_robot.links[0].name, self.psm2_robot.links[1].name)
                
                self.psm1_pub = rospy.Publisher('/dvrk/PSM1/set_base_frame', Pose, queue_size=10)
                self.psm2_pub = rospy.Publisher('/dvrk/PSM2/set_base_frame', Pose, queue_size=10)
            
                ecm_ee = self.ecm_kin.forward(q)
                ecm_base_frame = self.ecm_base.forward([]) 
                
                r_180_x = self.rotate('x', np.pi)
                r_90_z = self.rotate('z', -np.pi/2)
                
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
                
                for _ in range(1,10):
                    self.psm1_pub.publish(psm1_message)
                    self.psm2_pub.publish(psm2_message)
    #             ecm_message = pose_converter.PoseConv.to_pose_msg(ecm_base_frame)
            else:
                self.sub_ecm.unregister()
                self.psm1_pub.unregister()
                self.psm2_pub.unregister()
            
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
    
    class run_dvrk_console(QThread):
        def run(self):
            os.system('qlacloserelays')
            rospy.sleep(1)
            #home all the arms
#             rospy.Publisher('/dvrk/console/home', Empty, latch=True, queue_size=1).publish()
            os.system('rosrun dvrk_robot dvrk_console_json -j /home/dvrk/dev/Working\ Settings\ Files/Oct-2016/console-full-wsu.json')
        def kill(self):
            self.quit()
            
    def __init__(self):
        
        self.thread = None
        self.homing_thread = None
#         self.console_homing = self.run_dvrk_console()
#         self.console_homing.start()
        self.__mode__ = self.MODE.simulation 
        self.__control_mode__ = None
        
        # We have to initialize the node inside the main thread otherwise it would not work
        rospy.init_node('camera_control_node')
        
        widget_x = 20; widget_y = 30;
        
        self.a = QApplication(sys.argv) # Application handle
        self.w = QWidget() # Widget handle
        
        # Set window size.
        self.w.resize(320, 240)
         
        # Set window title
        self.w.setWindowTitle("Camera Options")
        
        widget_number = 1
        # Add home button
        self.btn_exit = QPushButton(self.w)
        self.btn_exit.setText('Home')
        self.btn_exit.clicked.connect(self.home)
        self.btn_exit.move(widget_x,widget_number * widget_y)
        # Add radio button for autocamera
        widget_number += 1
        self.radio_autocamera = QRadioButton('Autocamera', self.w)
        self.radio_autocamera.setToolTip('Activate Autocamera')
        self.radio_autocamera.clicked.connect(self.on_autocamera_select)
        self.radio_autocamera.resize(self.radio_autocamera.sizeHint())
        self.radio_autocamera.move(widget_x,widget_number*widget_y)
        
        # Add radio button for clutch and Go
        widget_number += 1
        self.radio_clutchNGo = QRadioButton('Clutch and Move', self.w)
        self.radio_clutchNGo.setToolTip('Activate camera clutch mechanism')
        self.radio_clutchNGo.clicked.connect(self.on_clutchNGo_select)
        self.radio_clutchNGo.resize(self.radio_clutchNGo.sizeHint())
        self.radio_clutchNGo.move(widget_x,widget_number*widget_y)
        
        # Add radio button for clutch and Go
        widget_number += 1
        self.radio_joystick = QRadioButton('Joystick Control', self.w)
        self.radio_joystick.setToolTip('Activate camera joystick mechanism')
        self.radio_joystick.clicked.connect(self.on_joystick_select)
        self.radio_joystick.resize(self.radio_joystick.sizeHint())
        self.radio_joystick.move(widget_x,widget_number*widget_y)
        
        # Add exit button
        widget_number += 1
        self.btn_exit = QPushButton(self.w)
        self.btn_exit.setText('Exit')
        self.btn_exit.clicked.connect(self.exit_program)
        self.btn_exit.move(widget_x,widget_number*widget_y)

        self.radio_simulation = QRadioButton('Simulation', self.w)
        self.radio_simulation.setToolTip('Simulation mode')
        self.radio_simulation.clicked.connect(self.on_simulation_select)
        self.radio_simulation.resize(self.radio_joystick.sizeHint())
        self.radio_simulation.move(6*widget_x,widget_y)
        self.radio_simulation.click()
        
        self.radio_hardware = QRadioButton('Hardware', self.w)
        self.radio_hardware.setToolTip('Hardware mode')
        self.radio_hardware.clicked.connect(self.on_hardware_select)
        self.radio_hardware.resize(self.radio_joystick.sizeHint())
        self.radio_hardware.move(11*widget_x,widget_y)
        
        camera_control_group = QButtonGroup(self.w)
        mode_group = QButtonGroup(self.w)
        
        camera_control_group.addButton(self.radio_autocamera)
        camera_control_group.addButton(self.radio_clutchNGo)
        camera_control_group.addButton(self.radio_joystick)
        
        mode_group.addButton(self.radio_simulation)
        mode_group.addButton(self.radio_hardware)
        # Show window
        self.w.show()
    
        sys.exit(self.a.exec_())
    
    @pyqtSlot()
    def home(self):
        self.homing_thread = self.thread_home_arms()
        self.homing_thread.start()
        
    @pyqtSlot()
    def exit_program(self):
        rospy.Publisher('/dvrk/console/power_off', Empty, latch=True, queue_size=1).publish()
        rospy.Publisher('/dvrk/console/teleop/enable', Bool, latch=True, queue_size=1).publish(Bool(False))
        if self.thread != None:
            self.thread.kill()
        if self.homing_thread != None:
            self.homing_thread.kill()
#         if self.console_homing != None:
#             self.console_homing.kill()
        sys.exit(self.a.exec_())
    
    @pyqtSlot()
    def on_autocamera_select(self):  
        self.__control_mode__ = self.node_name.autocamera
        self.start_node_handler(self.node_name.autocamera)
        #threading.Thread(target=self.start_node_handler, args=(self.node_name.autocamera))
        
    @pyqtSlot()
    def on_clutchNGo_select(self):
        self.__control_mode__ = self.node_name.clutchNGo
        self.start_node_handler(self.node_name.clutchNGo)
    
    @pyqtSlot()
    def on_joystick_select(self):
        self.__control_mode__ = self.node_name.joystick
        self.start_node_handler(self.node_name.joystick)
        
    @pyqtSlot()
    def on_simulation_select(self):
        if self.__mode__ != self.MODE.simulation :
            self.__mode__ = self.MODE.simulation
            if self.__control_mode__ != None:
                self.thread.kill()
                self.start_node_handler(self.__control_mode__)
    
    @pyqtSlot()
    def on_hardware_select(self):
        if self.__mode__ != self.MODE.hardware :
            self.__mode__ = self.MODE.hardware
            print(self.__control_mode__)
            if self.__control_mode__ != None:
                self.thread.kill()
                self.start_node_handler(self.__control_mode__)
    
    def start_node_handler(self, name=node_name.clutchNGo):
        msg = QMessageBox()
        msg.setText('running ' + name)
        retval = msg.exec_()
        
        if self.thread != None:
            self.thread.kill()

        rospy.Publisher('/dvrk/console/teleop/enable', Bool, latch=True, queue_size=1).publish(Bool(True))
                
        if name == self.node_name.clutchNGo :
            self.thread = self.thread_clutchNGo(self.__mode__)
            self.thread.start()
            
        elif name == self.node_name.autocamera:
            self.thread = self.thread_autocamera(self.__mode__)
            self.thread.start()
        elif name == self.node_name.joystick:
            self.thread = self.thread_joystick(self.__mode__)
            self.thread.start()
            
        
def main():
    camera_qt_gui()
    """
    node_handler = Autocamera_node_handler()
    node_handler.set_mode(node_handler.MODE.hardware)
    node_handler.debug_graphics(False)
    node_handler.spin()
    """
    

if __name__ == "__main__":
    main()
    print('hello')
    t1.kill()
