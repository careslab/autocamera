#!/usr/bin/env python

"""
@package camera_control_node.py

This file contains a GUI made using PyQt. This is used to power the system on, 
home the arms, switch between simulation and hardware and choose a camera control 
method. It also provides the capability to record all the movements.
"""
#QApplication
from __common_imports__ import *

import sys

from autocamera_algorithm import Autocamera
import os
import tf

import threading


from Crypto.Signature.PKCS1_PSS import PSS_SigScheme

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QDialog, QLineEdit, QPushButton, QMessageBox

#from camera_control_gui  import Ui_Dialog
#from PyQt5.QtCore import pyqtSlot, SIGNAL, pyqtSignal
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
#from PyQt5.QtCore import Qt
#from PyQt5 import QtGui
#from PyQt5.QtGui import *
#from PyQt5.Qt import QObject


import configparser
import camera_control_gui
from camera_control_gui import Ui_Dialog
import rosbag

import pexpect
import time
from time import strptime

from clutchless_system import ClutchlessSystem    
from teleoperation import TeleopClass    
from clutch_control import ClutchControl
from joystick_camera_control import JoystickClass
from oculus_camera_control import OculusClass
       
class bag_writer:
    class MODE:
        """
            simulation mode: Use the hardware for the master side, 
                            and simulation for the ECM
            hardware mode: Use hardware for both the master side,
                            and the ECM
        """
        simulation = "SIMULATION"
        hardware = "HARDWARE"
        
    def __init__(self, arm_names, bag_name='test', mode = MODE.simulation, recording_dir='recordings'):
        if type(arm_names) == type(""):
            arm_names = [arm_names]
            
        self.__mode__ = mode
        
        # initialize bags 
        dir = '{}/'.format(recording_dir); dir = dir.__str__()
        if not os.path.exists(dir):
            os.system( ('mkdir -p '+ dir ).__str__())
        
        record_command = """rosbag record /dvrk/MTML/state_joint_current /dvrk/MTMR/state_joint_current /dvrk/PSM1/state_joint_current /dvrk/PSM2/state_joint_current /dvrk/ECM/state_joint_current /dvrk/footpedals/clutch /dvrk/footpedals/camera /dvrk/footpedals/coag /joy /camera1/usb_cam_left/image_raw/compressed /camera2/usb_cam_right/image_raw/compressed  --lz4 --duration=600 -O {}""".format(dir + bag_name + '_sim.bag' )
        print(record_command)
        self.p = pexpect.spawn( record_command)
        return
        self.bag_sim = rosbag.Bag(dir + bag_name + '_sim.bag', 'w')
#         self.bag_hw = rosbag.Bag(dir + bag_name + '_hw.bag', 'w')
#         self.bag_hw.compression=rosbag.Compression.BZ2
        self.bag_sim.compression=rosbag.Compression.BZ2
        self.bag_sim.chunk_threshold=6000000
#         self.bag_hw.chunk_threshold=60000000
        
        self.arm_names = arm_names
        # The topics we want to record for each arm:
#         Anything with state_joint_current
#         /dvrk/footpedals/clutch
#         /dvrk/footpedals/camera
#         /dvrk/footpedals/coag
#         The video feed
        
#         self.topics = '/dvrk/{}/state_joint_current'.format(ARM_NAME)
        self.topics = { arm_name:'/dvrk/{}/state_joint_current'.format(arm_name) for arm_name in self.arm_names}
        self.out_topics_hw = {arm_name : '/dvrk/{}/set_position_joint'.format(arm_name) for arm_name in self.arm_names}
        self.out_topics_sim = {arm_name : '/dvrk_{}/joint_states_robot'.format(arm_name.lower()) for arm_name in self.arm_names}
        
        self.sub_footpedal_clutch = rospy.Subscriber('/dvrk/footpedals/clutch', Joy, self.cb_clutch)
        self.sub_footpedal_camera = rospy.Subscriber('/dvrk/footpedals/camera', Joy, self.cb_coag)
        self.sub_footpedal_coag = rospy.Subscriber('/dvrk/footpedals/coag', Joy, self.cb_camera)
        
        self.sub_image_left = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.cb_image_left)
        # Add another subscriber here for image:
        
        for arm_name in self.arm_names:
            exec("self.sub_{} = rospy.Subscriber('{}', JointState, self.cb_{})".format(arm_name, self.topics[arm_name], arm_name))
            rospy.timer.sleep(.1)
    def set_mode(self, mode):
        self.__mode__ = mode
    
    def shutdown(self):
        # unregister all subscribers
        self.p.sendcontrol('c')
        self.p.sendintr()
        self.p.close()
        print('recording finished')
        return
        for arm_name in self.arm_names:
            eval("self.sub_{}.unregister()".format(arm_name))
            rospy.timer.sleep(.1)
            
        self.sub_footpedal_camera.unregister()
        self.sub_footpedal_clutch.unregister()
        self.sub_footpedal_coag.unregister()
        self.sub_image_left.unregister()
        
#         self.bag_hw.close()
        self.bag_sim.close()
        
        
        
    def spin(self):
        self.p.wait()
        rospy.spin()
    
    def run_once(f):
        def wrapper(*args, **kwargs):
            if not wrapper.has_run:
                wrapper.has_run = True
                return_value = f(*args, **kwargs)
                wrapper.has_run = False
                return return_value
        wrapper.has_run = False
        return wrapper
        
    @run_once    
    def cb_generic(self, topic, msg):
        try:
            self.bag_sim.write(topic, msg)
#             self.bag_hw.write(topic, msg)
        except Exception:
            print("there was an error")
    
    def cb_image_left(self, msg):
        self.cb_generic('/usb_cam/image_raw/compressed', msg)
                
    def cb_clutch(self, msg):
        self.cb_generic('/dvrk/footpedals/clutch', msg)
    
    def cb_camera(self, msg):
        self.cb_generic('/dvrk/footpedals/camera', msg)

    def cb_coag(self, msg):
        self.cb_generic('/dvrk/footpedals/coag', msg)
    
    def cb_MTML(self, msg):
        self.cb('MTML', msg)
    def cb_MTMR(self, msg):
        self.cb('MTMR', msg)
    def cb_PSM1(self, msg):
        self.cb('PSM1', msg)
    def cb_PSM2(self, msg):
        self.cb('PSM2', msg)
    def cb_ECM(self, msg):
        self.cb('ECM', msg)

    def cb(self,arm_name, msg):
#         try:
        self.cb_generic(self.out_topics_sim[arm_name], msg) # record the msg
#             self.bag_hw.write(self.out_topics_hw[arm_name], msg) # record the msg
#         except Exception as e:
#             print("there was an error in {}: {}".format(arm_name, e)) 
    
    # This is the destructor for this python class    
#     def __del__(self):
#         self.bag_sim.close()
#         self.bag_hw.close()
    

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
        self.headsensor_active = False
        self.repositioning_clutch_active = False
        
        # For forward and inverse kinematics
        self.__ecm_robot__ = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        self.__ecm_kin__ = KDLKinematics(self.__ecm_robot__, self.__ecm_robot__.links[0].name, self.__ecm_robot__.links[-1].name)
        self.__psm1_robot__ = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        self.__psm1_kin__ = KDLKinematics(self.__psm1_robot__, self.__psm1_robot__.links[0].name, self.__psm1_robot__.links[-1].name)
        self.__mtml_robot__ = URDF.from_parameter_server('/dvrk_mtml/robot_description')
        self.__mtml_kin__ = KDLKinematics(self.__mtml_robot__, self.__mtml_robot__.links[0].name, self.__mtml_robot__.links[-1].name)
        self.__mtmr_robot__ = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
        self.__mtmr_kin__ = KDLKinematics(self.__mtmr_robot__, self.__mtmr_robot__.links[0].name, self.__mtmr_robot__.links[-1].name)
        
        
        # For camera clutch control    
        self.camera_clutch_pressed = False        
        self.ecm_manual_control_lock_mtml_msg = None
        self.ecm_manual_control_lock_ecm_msg = None
        self.mtml_start_position = None
        self.mtml_end_position = None
        
        # Get zoom parameters from config file
        self.config = configparser.ConfigParser()
        self.config_file = '../config/autocamera.conf'
        self.config.read(self.config_file)
        
        inner = 0.08
        outer = 0.08
        
        self.set_autocamera_params(inner, outer)
        
        self.initialize_psms_initialized = 30
        self.__DEBUG_GRAPHICS__ = False
        
        
    def __init_nodes__(self):
        self.hw_ecm = robot('ECM')
        self.__hw_psm1__ = robot('PSM1')
        self.__hw_psm2__ = robot('PSM2')
            
        #rospy.init_node('autocamera_node')
        
        self.logerror("start", debug=True)
        
        # Publishers to the simulation
        self.pub_ecm = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=1, tcp_nodelay=True)
        self.__pub_psm1__ = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=1, tcp_nodelay=True)
        self.__pub_psm2__ = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=1, tcp_nodelay=True)
        
        # Get the joint angles from the simulation
        self.sub_ecm_sim = rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.add_ecm_jnt, queue_size=1, tcp_nodelay=True)
        self.sub_caminfo = rospy.Subscriber('/fakecam_node/camera_info', CameraInfo, self.get_cam_info, queue_size=1, tcp_nodelay=True)
        
        try:
            self.sub_psm1_sim.unregister()
            self.sub_psm2_sim.unregister()
            self.sub_psm1_hw.unregister()
            self.sub_psm2_hw.unregister()
        except Exception:
            pass
        if self.__AUTOCAMERA_MODE__ == self.MODE.hardware :
            # Get the joint angles from the hardware and move the simulation from hardware
            self.sub_psm1_hw = rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, self.add_psm1_jnt, queue_size=1, tcp_nodelay=True)
            self.sub_psm2_hw = rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, self.add_psm2_jnt, queue_size=1, tcp_nodelay=True)
            self.sub_ecm_sim = rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.ecm_cb_hw, queue_size=1, tcp_nodelay=True)
            
            # subscribe to head sensor
            self.sub_headsensor = rospy.Subscriber('/dvrk/footpedals/coag', Joy, self.__headsensor_cb__ , queue_size=1, tcp_nodelay=True)
            self.sub_repositioning_clutch = rospy.Subscriber('/dvrk/footpedals/clutch', Joy, self.repositioning_clutch_cb , queue_size=1, tcp_nodelay=True)
            
        elif self.__AUTOCAMERA_MODE__ == self.MODE.simulation:
            # Get the joint angles from the simulation
            self.sub_psm1_sim = rospy.Subscriber('/dvrk_psm1/joint_states', JointState, self.add_psm1_jnt, queue_size=1, tcp_nodelay=True)
            self.sub_psm2_sim = rospy.Subscriber('/dvrk_psm2/joint_states', JointState, self.add_psm2_jnt, queue_size=1, tcp_nodelay=True)
            
            # If hardware is connected, subscribe to it and set the psm joint angles in the simulation from the hardware
#             self.sub_psm1_hw = rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, self.add_psm1_jnt_from_hw)
#             self.sub_psm2_hw = rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, self.add_psm2_jnt_from_hw)
            
            
        # Get the joint angles from MTM hardware
        ##rospy.Subscriber('/dvrk/MTML/position_joint_current', JointState, self.__mtml_cb__)
    #     rospy.Subscriber('/dvrk/MTMR/position_joint_current', JointState, add_psm1_jnt)
        
        # Detect whether or not the camera clutch is being pressed
##         rospy.Subscriber('/dvrk/footpedals/camera', Bool, self.camera_clutch_cb)
        
        # Move the hardware from the simulation
    #     rospy.Subscriber('/dvrk_psm1/joint_states', JointState, self.move_psm1)
    #     rospy.Subscriber('/dvrk_psm2/joint_states', JointState, self.move_psm2)
        
        if self.__DEBUG_GRAPHICS__ == True:
            # Subscribe to fakecam images
            self.sub_fake_image_left = rospy.Subscriber('/fakecam_node/fake_image_left', Image, self.left_image_cb, queue_size=1)
            self.sub_fake_image_right = rospy.Subscriber('/fakecam_node/fake_image_right', Image, self.right_image_cb, queue_size=1)
         
        # Publish images
        self.pub_image_left = rospy.Publisher('autocamera_image_left', Image, queue_size=1)
        self.pub_image_right = rospy.Publisher('autocamera_image_right', Image, queue_size=1)

    def shutdown(self):
        try:
            if self.__DEBUG_GRAPHICS__ == True:
                self.sub_fake_image_left.unregister()
                self.sub_fake_image_right.unregister()
            if self.__AUTOCAMERA_MODE__ == self.MODE.simulation:
                self.sub_psm1_sim.unregister()
                self.sub_psm2_sim.unregister()
            if self.__AUTOCAMERA_MODE__ == self.MODE.hardware:
                self.sub_headsensor.unregister()
                self.sub_repositioning_clutch.unregister()
            
            self.sub_ecm_sim.unregister()
            self.sub_psm1_hw.unregister()
            self.sub_psm2_hw.unregister()
            self.sub_caminfo.unregister()
            
            self.pub_image_left.unregister()
            self.pub_image_right.unregister()
            self.pub_ecm.unregister()
            self.__pub_psm1__.unregister()
            self.__pub_psm2__.unregister()
            
            self.hw_ecm.unregister()
            self.__hw_psm1__.unregister()
            self.__hw_psm2__.unregister()
            print( "Shutting down " + self.__class__.__name__)
            
        except Exception:
            print("couldn't unregister all the topics")
#         rospy.signal_shutdown('shutting down Autocamera')
        
    # This needs to be run before anything can be expected
    def spin(self):
        self.__init_nodes__() # initialize all the nodes, subscribers and publishers
        rospy.spin()
     
    def set_autocamera_params(self, inner_zone, dead_zone):
        self.autocamera.zoom_deadzone_radius = dead_zone
        self.autocamera.zoom_innerzone_radius = inner_zone
        
        #self.config.set('ZOOM', 'inner_zone', inner_zone.__str__())
        #self.config.set('ZOOM', 'dead_zone', dead_zone.__str__())
        
        #with open(self.config_file,'w') as cfile:
        #    self.config.write(cfile)
        
    def get_autocamera_params(self):
        return self.autocamera.zoom_deadzone_radius, self.autocamera.zoom_innerzone_radius
         
    def debug_graphics(self, has_graphics):
        self.__DEBUG_GRAPHICS__ = has_graphics

    def image_cb(self, image_msg, camera_name):
        image_pub = {'left':self.pub_image_left, 'right':self.pub_image_right}[camera_name]
        
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
            
            inner_radius = self.autocamera.zoom_innerzone_radius 
            deadzone_radius = self.autocamera.zoom_deadzone_radius + self.autocamera.zoom_innerzone_radius
            
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
            
    def logerror(self, msg, debug = False):
        if self.DEBUG or debug:
            rospy.logerr(msg)
            
    def ecm_manual_control_lock(self, msg, fun):
        """!
            Switch between manual control and automatic control of the ECM
        """
        
        if fun == 'ecm':
            self.ecm_manual_control_lock_ecm_msg = msg
        elif fun == 'mtml':
            self.ecm_manual_control_lock_mtml_msg = msg
        
        if self.ecm_manual_control_lock_mtml_msg is not None and self.ecm_manual_control_lock_ecm_msg is not None:
            self.ecm_manual_control(self.ecm_manual_control_lock_mtml_msg, self.ecm_manual_control_lock_ecm_msg)
            self.ecm_manual_control_lock_mtml_msg = None
            self.ecm_manual_control_lock_ecm_msg = None
    
    
    def ecm_manual_control(self, mtml_msg, ecm_msg):
        # TODO: Find forward kinematics from mtmr and ecm. Move mtmr. Find the movement vector. Add it to the 
        # ecm position, use inverse kinematics and move the ecm.
            
        self.logerror("mtml" + self.__mtml_kin__.get_joint_names().__str__())
        start_coordinates,_ = self.__mtml_kin__.FK( self.mtml_start_position.position[:-1]) # Returns (position, rotation)
        end_coordinates,_ = self.__mtml_kin__.FK(mtml_msg.position[:-1])
        
        diff = np.subtract(end_coordinates, start_coordinates)
        self.logerror("diff = " + diff.__str__())
        
        # Find the ecm 3d coordinates, add the 'diff' to it, then do an inverse kinematics
        ecm_coordinates,_ = self.__ecm_kin__.FK(ecm_msg.position[0:2] + ecm_msg.position[-2:]) # There are a lot of excessive things here that we don't need
        ecm_pose = self.__ecm_kin__.forward(ecm_msg.position[0:2] + ecm_msg.position[-2:])
        
        # Figure out the new orientation and position to be used in the inverse kinematics
        b,_ = self.__ecm_kin__.FK([ecm_msg.position[0],ecm_msg.position[1],.14,ecm_msg.position[3]])
        keyhole, _ = self.__ecm_kin__.FK([0,0,0,0])
        ecm_current_direction = b-keyhole
        new_ecm_coordinates = np.add(ecm_coordinates, diff)
        ecm_new_direction = new_ecm_coordinates - keyhole
        r = self.autocamera.find_rotation_matrix_between_two_vectors(ecm_current_direction, ecm_new_direction)
        
        ecm_pose[0:3,0:3] =  r* ecm_pose[0:3,0:3] 
        ecm_pose[0:3,3] = new_ecm_coordinates
        new_ecm_joint_angles = self.__ecm_kin__.inverse(ecm_pose)
        
        new_ecm_msg = ecm_msg; new_ecm_msg.position = new_ecm_joint_angles; new_ecm_msg.name = self.__ecm_kin__.get_joint_names()
        
        self.logerror("ecm_new_direction " + ecm_new_direction.__str__())
        self.logerror('ecm_coordinates' + ecm_coordinates.__str__())
        self.logerror("ecm_pose " + ecm_pose.__str__())
        self.logerror("new_ecm_joint_angles " + new_ecm_joint_angles.__str__())
        self.logerror("new_ecm_msg" + new_ecm_msg.__str__())
        
        self.pub_ecm.publish(new_ecm_msg)
        
    
    def __headsensor_cb__(self, msg):
        if msg.buttons[0] == 1:
            self.headsensor_active = True
        else:
            self.headsensor_active = False 
    
    def repositioning_clutch_cb(self, msg):
        if msg.buttons[0] == 1:
            self.repositioning_clutch_active = True
        else:
            self.repositioning_clutch_active = False 
                   
    def camera_clutch_cb(self, msg):
        self.camera_clutch_pressed = msg.data
        self.logerror('Camera Clutch : ' + self.camera_clutch_pressed.__str__())
        
    
    def __mtml_cb__(self, msg):
        if self.camera_clutch_pressed :
            if self.mtml_start_position is  None:
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
        
        self.__hw_psm1__.move_joint_list(msg.position, interpolate=False)
    
    def move_psm2(self, msg):
        jaw = msg.position[-3]
        msg = self.autocamera.extract_positions(msg, 'psm2')
        
        msg.name = msg.name + ['jaw']
        msg.position = msg.position + [jaw]
        
        self.__hw_psm2__.move_joint_list(msg.position, interpolate=False)
    
    # ecm callback    
    
    def ecm_cb_hw(self, msg):
        msg.name = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
        self.pub_ecm.publish(msg)
        
    def add_ecm_jnt(self, msg):
        if self.camera_clutch_pressed == False and msg is not None:
            if self.__MOVE_ECM_WITH_SLIDERS__ == False:
#                 if self.__AUTOCAMERA_MODE__ == self.MODE.hardware:
#                     msg.name = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
                self.add_jnt('ecm', msg)
            else:
                temp = list(msg.position[:2]+msg.position[-2:])
                r = self.hw_ecm.move_joint_list(temp, interpolate=first_run)
                if r == True:
                    self.first_run = False
        else:
            self.ecm_manual_control_lock(msg, 'ecm')
       
    
    def add_psm1_jnt_from_hw(self, msg):
        if self.camera_clutch_pressed == False and msg is not None:
            if self.__AUTOCAMERA_MODE__ == self.MODE.simulation:
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.__pub_psm1__.publish(msg)

    def add_psm2_jnt_from_hw(self, msg):
        if self.camera_clutch_pressed == False and msg is not None:
            if self.__AUTOCAMERA_MODE__ == self.MODE.simulation:
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.__pub_psm2__.publish(msg)
                
    # psm1 callback    
    def add_psm1_jnt(self, msg):
        if self.camera_clutch_pressed == False and msg is not None:
            # We need to set the names, otherwise the simulation won't move
            if self.__AUTOCAMERA_MODE__ == self.MODE.hardware:
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.__pub_psm1__.publish(msg)
            self.add_jnt('psm1', msg)
                
         
    # psm2 callback    
    def add_psm2_jnt(self, msg):
        if self.camera_clutch_pressed == False and msg is not None:
            if self.__AUTOCAMERA_MODE__ == self.MODE.hardware :
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.__pub_psm2__.publish(msg)
            self.add_jnt('psm2', msg)
                
    def add_jnt(self, name, msg):
        self.joint_angles[name] = msg
        if self.headsensor_active == False or self.repositioning_clutch_active == True:
            return
        if not None in self.joint_angles.values():
            if self.initialize_psms_initialized>0 and self.__AUTOCAMERA_MODE__ == self.MODE.simulation:        
                self.initialize_psms()
                time.sleep(.01)
                self.initialize_psms_initialized -= 1
            try:
                old_zoom = self.joint_angles['ecm'].position[-2]
                jnt_msg = 'error'
                jnt_msg = self.autocamera.compute_viewangle(self.joint_angles, self.cam_info)
                
                self.pub_ecm.publish(jnt_msg)
                
                jnt_msg.position = [ round(i,4) for i in jnt_msg.position]
                if len(jnt_msg.position) != 4 or len(jnt_msg.name) != 4 :
                    return
                #return # stop here until we co-register the arms
                if self.__AUTOCAMERA_MODE__ == self.MODE.hardware:
                    pos = list(jnt_msg.position)
                    result = False
                    if self.first_run:
                        result = self.hw_ecm.move_joint_list(pos, index=[0,1,2,3], interpolate=self.first_run)
                    else:
                        new_zoom = pos[2]
                        u = (old_zoom + new_zoom)/2.0
#                         del pos[2]
                        
                        # Move everything besides the zoom
                        result = self.hw_ecm.move_joint_list(pos, index=[0,1,2,3], interpolate=self.first_run)
                        
#                         x = np.arange(old_zoom, new_zoom, np.sign(new_zoom - old_zoom) * 0.00001)
#                         y = ( (1/np.sqrt(2*np.pi))*np.e**((-((x-u)**2)/2)))
#                         y2 = y - y[0]
#                         x2 = x + y2
#                         print('old_zoom = {}\nnew_zoom = {}\nx={}\ny={}\n'.format(old_zoom, new_zoom, x2,y2))
#                         print('len(y) = {}'.format(len(y2)))
#                         for z in x:
#                             z = float(z)
#                             r = self.hw_ecm.move_joint_list([z], index = [2], interpolate=False)
#                             print('r = {}'.format(r))
#                     pos = [jnt_msg.position[2]]
#                     result= self.hw_ecm.move_joint_list(pos, index=[2], interpolate=self.first_run)
                    # Interpolate the insertion joint individually and the rest without interpolation
#                     result = self.hw_ecm.move_joint_list(pos, index=[2], interpolate=self.first_run)
                    if result == True:
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
        

    def left_image_cb(self, image_msg):
        self.image_cb(image_msg, 'left')    
        
    def right_image_cb(self, image_msg):
        self.image_cb(image_msg, 'right')
    
    
    def initialize_psms(self):
        if self.__AUTOCAMERA_MODE__ == self.MODE.simulation : 
            msg = JointState()
            msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
            msg.position = [0.84 , -0.65, 0.10, 0.00, 0.00, 0.00, 0.00]
            self.__pub_psm1__.publish(msg)
            msg.position = [-0.84 , -0.53, 0.10, 0.00, 0.00, 0.00, 0.00]
            self.__pub_psm2__.publish(msg)
            self.logerror('psms initialized!')
            
    
    def set_mode(self, mode):
        """ Values:
            MODE.simulation
            MODE.hardware
        """
        self.__AUTOCAMERA_MODE__ = mode
        
        


           

class camera_qt_gui(QDialog, camera_control_gui.Ui_Dialog):
    
    class node_name:
        clutchNGo = 'clutch_control'
        autocamera = 'Autocamera'
        joystick = 'joystick'
        teleop = 'teleop'
        oculus = 'oculus'
        clutchless = 'clutchless'
        
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
    
    
    class thread_teleop(QThread):
        def __init__(self, mode):
            super(QThread, self).__init__()
            self.node_handler = None
            self.__mode__ = mode
        def run(self):
            self.node_handler = TeleopClass()
            print('\nRunning {} in {}\n'.format("Teleop",self.__mode__))
            self.node_handler.set_mode(self.__mode__)
            self.node_handler.spin()
        
        def kill(self):
            if self.node_handler is not None:
                self.node_handler.shutdown()
                self.quit()
                
        def home(self):
            print('Teleop homing')
            self.node_handler.rehome()
            
        def pause(self):
            self.node_handler.pause()
        def resume(self):
            self.node_handler.enable_teleop()
        
        def lock_mtm_orientations(self, msg):
            self.node_handler.lock_mtm_orientations()
            
    class thread_bag_writer(QThread):
        def __init__(self, arm_names, file_name, recording_dir, mode):
            super(QThread, self).__init__()
            self.node_handler = None
            self.__mode__ = mode
            self.arm_names = arm_names
            self.file_name = file_name
            self.recording_dir = recording_dir
            
        def run(self):
            self.node_handler = bag_writer(self.arm_names, self.file_name, recording_dir=self.recording_dir)
            self.node_handler.spin()
        
        def kill(self):
            if self.node_handler is not None:
                self.node_handler.shutdown()
                self.quit()
                
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
            if self.node_handler is not None:
                self.node_handler.shutdown()
                self.quit()
            
    class thread_clutchNGo(QThread):
        def __init__(self, mode, teleop_thread):
            super(QThread, self).__init__()
            self.node_handler = None
            self.__mode__ = mode
            self.teleop_thread = teleop_thread
            
        def run(self):
            self.node_handler = ClutchControl(self.teleop_thread)
            print('\nRunning {} in {}\n'.format("Clutch and Move",self.__mode__))
            self.node_handler.set_mode(self.__mode__)
            self.node_handler.spin()
        
        def kill(self):
            if self.node_handler is not None:
                self.node_handler.shutdown()
                self.quit()
                
    class thread_joystick(QThread):
        def __init__(self, mode):
            super(QThread, self).__init__()
            self.node_handler = None
            self.__mode__ = mode
        def run(self):
            self.node_handler = JoystickClass()
            print('\nRunning {} in {}\n'.format("JoystickClass Control",self.__mode__))
            self.node_handler.set_mode(self.__mode__)
            self.node_handler.spin()
        
        def kill(self):
            if self.node_handler is not None:
                self.node_handler.shutdown()
                self.quit()
    
    class thread_oculus(QThread):
        def __init__(self, mode):
            super(QThread, self).__init__()
            self.node_handler = None
            self.__mode__ = mode
        def run(self):
            self.node_handler = OculusClass()
            print('\nRunning {} in {}\n'.format("OculusClass Control",self.__mode__))
            self.node_handler.set_mode(self.__mode__)
            self.node_handler.spin()
        
        def kill(self):
            if self.node_handler is not None:
                self.node_handler.shutdown()
                self.quit()
    
    class thread_clutchless(QThread):
        def __init__(self, mode):
            super(QThread, self).__init__()
            self.node_handler = None
            self.__mode__ = mode
        def run(self):
            self.node_handler = ClutchlessSystem()
            print('\nRunning {} in {}\n'.format("clutchless system",self.__mode__))
            self.node_handler.set_mode(self.__mode__)
            self.node_handler.spin()
        
        def kill(self):
            if self.node_handler is not None:
                self.node_handler.shutdown()
                self.quit()
                
        def home(self):
            print('clutchless homing')
            self.node_handler.rehome()
            
        def pause(self):
            self.node_handler.pause()
        def resume(self):
            self.node_handler.enable_teleop()
        
        def lock_mtm_orientations(self, msg):
            self.node_handler.lock_mtm_orientations()
            
    class thread_timer(QThread):
        trigger = pyqtSignal(int)
        
        def __init__(self, parent=None):
            super(QThread, self).__init__(parent)
    
        def setup(self, thread_no):
            self.thread_no = thread_no
    
        def run(self):
            while True:
                time.sleep(1)  # random sleep to imitate working
                self.trigger.emit(self.thread_no)
        def kill(self):
            self.trigger.disconnect()
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
            self.ecm = robot('ECM')
            
        def run(self):
            self.home()
            
        def home(self):
            r = self.ecm.move_joint_list([0.0,0.0,0.0,0.0],[0,1,2,3], interpolate=True)
            
            
        def kill(self):
#             if self.is_running == False:
            self.quit()
            self.ecm.unregister()
    
    class run_dvrk_console(QThread):
        def run(self):
            os.system('qlacloserelays')
            rospy.sleep(1)
            #home all the arms
#             rospy.Publisher('/dvrk/console/home', Empty, latch=True, queue_size=1).publish()
            os.system('rosrun dvrk_robot dvrk_console_json -j /home/dvrk/dev/Working\ Settings\ Files/Oct-2016/console-full-wsu.json')
        def kill(self):
            self.quit()
            
    def __init__(self, parent=None):
        super(camera_qt_gui, self).__init__()
        self.setupUi(self)
#         
        self.thread = None
        self.thread_tel = None
        self.homing_thread = None

        self.__mode__ = self.MODE.simulation 
        self.__control_mode__ = None
         
        # We have to initialize the node inside the main thread otherwise it would not work
        rospy.init_node('camera_control_node')
        
        self.pushButtonHome.clicked.connect(self.home)
        self.pushButtonPowerOn.clicked.connect(self.power_on)
        self.pushButtonPowerOff.clicked.connect(self.power_off)
        self.pushButtonReset.clicked.connect(self.reset)
        self.pushButtonExit.clicked.connect(self.exit_program)
        
        self.radioButtonTeleop.clicked.connect(self.on_teleop_select)
        self.radioButtonAutocamera.clicked.connect(self.on_autocamera_select)
        self.radioButtonClutchAndMove.clicked.connect(self.on_clutchNGo_select)
        self.radioButtonJoystick.clicked.connect(self.on_joystick_select)
        self.radioButtonOculus.clicked.connect(self.on_oculus_select)
        self.radioButtonClutchless.clicked.connect(self.on_clutchless_select)
        
        self.horizontalSliderInnerzone.valueChanged[int].connect(self.horizontalSliderInnerzoneCb)
        self.horizontalSliderDeadzone.valueChanged[int].connect(self.horizontalSliderDeadzoneCb)
        
        self.radioButtonSimulation.setChecked(True)
        self.radioButtonSimulation.clicked.connect(self.on_simulation_select)
        self.radioButtonHardware.clicked.connect(self.on_hardware_select)
        
        self.groupBoxAutocameraParams.setVisible(False)
        
        self.pushButtonHome.setEnabled(False)
        self.pushButtonPowerOff.setEnabled(False)
        self.pushButtonRecord.setEnabled(False)
        self.pushButtonReset.setEnabled(False)
        self.spinBoxSubjectNumber.setEnabled(False)
        self.spinBoxPatternNumber.setEnabled(False)
        self.groupBoxOperationMode.setEnabled(False)
        self.groupBoxCameraControlMethod.setEnabled(False)
        
        self.recording = False
        self.pushButtonRecord.clicked.connect(self.on_record)
        
        
        try:
            # Get recording parameters from config file
            self.config = configparser.ConfigParser()
            self.config_file = '../config/autocamera.conf'
            self.config.read(self.config_file)
            self.recording_dir = self.config.get('RECORDING', 'directory')
            if os.path.exists(self.recording_dir):
                subject_number = len(os.listdir(self.recording_dir)) 
                self.spinBoxSubjectNumber.setValue(subject_number)
        except Exception:
            pass

        
    @pyqtSlot()
    def on_record(self):
        if self.recording == False:
            
            folder_name = "subject" + str(self.spinBoxSubjectNumber.text())
            number = 1
            if os.path.exists('{}/'.format(self.recording_dir)+folder_name):
                number = 1+len(os.listdir('{}/{}'.format(self.recording_dir, folder_name)));
                subject_number = len(os.listdir(self.recording_dir)) 
                self.spinBoxSubjectNumber
                self.labelSubjectInfo.setText('Task # ' + str(number))
                self.labelSubjectInfo.setStyleSheet("color: brown")
            else:
                self.labelSubjectInfo.setText('')
            self.recording = True
            self.spinBoxSubjectNumber.setEnabled(False)
            self.spinBoxPatternNumber.setEnabled(False)
            self.pushButtonRecord.setStyleSheet("background-color: red")
            self.pushButtonRecord.setText('Stop Recording')
            arm_names = ['MTML', 'MTMR', 'PSM1', 'PSM2', 'ECM']
            rdir = self.recording_dir+ '/' + folder_name 
            pattern = str(self.spinBoxPatternNumber.text())
            file_name = "{}_{}_pattern{}".format(number, self.__control_mode__, pattern)
            
            self.bag_writer = self.thread_bag_writer(arm_names, file_name, recording_dir=rdir, mode=self.MODE.hardware)
            self.bag_writer.start()
            
            self.recording_start_time = time.time()
            
            
            
                
            self.timer_thread = self.thread_timer(self)
            self.timer_thread.setup(0)
            self.timer_thread.trigger.connect(self.update_timer)
            self.timer_thread.start()
        else:
            self.recording = False
            self.pushButtonRecord.setStyleSheet("background-color: ")
            self.pushButtonRecord.setText('Record')
            self.bag_writer.kill()
            self.spinBoxSubjectNumber.setEnabled(True)
            self.spinBoxPatternNumber.setEnabled(True)
            self.timer_thread.kill()
            self.labelTimer.setStyleSheet("color: black")
            
    
    @pyqtSlot(int)            
    def update_timer(self, thread_no):
        import datetime
        
        s = int(time.time() - self.recording_start_time)
        m, s = divmod(s, 60)
        h, m = divmod(m, 60)
        self.labelTimer.setText( '{}:{}:{}'.format(h,m,s))
        self.labelTimer.setStyleSheet("color: red")
    
    @pyqtSlot()
    def horizontalSliderInnerzoneCb(self):
        self.set_autocamera_params()
        
    @pyqtSlot()
    def horizontalSliderDeadzoneCb(self):
        self.set_autocamera_params()
    
    def set_autocamera_params(self):
        inner_zone = self.horizontalSliderInnerzone.value()/100.0
        dead_zone = self.horizontalSliderDeadzone.value()/100.0
        print(inner_zone, dead_zone)
        self.labelInnerzoneValue.setText(inner_zone.__str__())
        self.labelDeadzoneValue.setText(dead_zone.__str__())

        self.thread.node_handler.set_autocamera_params(inner_zone, dead_zone)
    def get_autocamera_params(self):
        while self.thread.node_handler is  None:
                pass
            
        dead_zone, inner_zone = self.thread.node_handler.get_autocamera_params()
        self.horizontalSliderDeadzone.setValue(dead_zone*100)
        self.horizontalSliderInnerzone.setValue(inner_zone*100)
        
    @pyqtSlot()
    def home(self):
        self.groupBoxOperationMode.setEnabled(True)
        self.groupBoxCameraControlMethod.setEnabled(True)
        self.pushButtonRecord.setEnabled(True)
        self.pushButtonReset.setEnabled(True)
        self.spinBoxSubjectNumber.setEnabled(True)
        self.spinBoxPatternNumber.setEnabled(True)
        
        self.homing_thread = self.thread_home_arms()
        self.homing_thread.start()
        self.homing_thread.home()
        

    @pyqtSlot()
    def power_on(self):
        self.pushButtonHome.setEnabled(True)
        self.pushButtonExit.setEnabled(False)
        self.pushButtonPowerOff.setEnabled(True)
        self.pushButtonPowerOn.setEnabled(False)
        self.pushButtonReset.setEnabled(True)
        rospy.Publisher('/dvrk/console/home', Empty, latch=True, queue_size=1).publish()
    
    @pyqtSlot()
    def power_off(self):
        self.pushButtonHome.setEnabled(False)
        self.pushButtonReset.setEnabled(False)
        self.pushButtonExit.setEnabled(True)
        self.pushButtonPowerOff.setEnabled(False)
        self.pushButtonPowerOn.setEnabled(True)
        self.groupBoxOperationMode.setEnabled(False)
        self.groupBoxCameraControlMethod.setEnabled(False)
        
        rospy.Publisher('/dvrk/console/power_off', Empty, latch=True, queue_size=1).publish(Empty())
                
    @pyqtSlot()
    def reset(self):
        self.radioButtonTeleop.setChecked(True)
        
        if self.thread is not None:
            self.thread.kill()
        if self.thread_tel is not None:
            self.thread_tel.home()
        else:
            self.thread_tel = self.start_node_handler(self.node_name.teleop)
            self.thread_tel.start()
        if self.homing_thread is not None:
            self.homing_thread.home()
        else:
            self.home()
        
        self.thread = None
        
        
        
    @pyqtSlot()
    def exit_program(self):
        rospy.Publisher('/dvrk/console/power_off', Empty, latch=True, queue_size=1).publish()
        rospy.Publisher('/dvrk/console/teleop/enable', Bool, latch=True, queue_size=1).publish(Bool(False))
        if self.thread is not None:
            self.thread.kill()
        if self.homing_thread is not None:
            self.homing_thread.kill()
#         if self.console_homing != None:
#             self.console_homing.kill()
        sys.exit(0)
    
    @pyqtSlot()
    def on_teleop_select(self):
        self.__control_mode__ = self.node_name.teleop
        self.start_node_handler(self.node_name.teleop)
        
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
    def on_oculus_select(self):
        self.__control_mode__ = self.node_name.oculus
        self.start_node_handler(self.node_name.oculus)

    @pyqtSlot()
    def on_clutchless_select(self):
        self.__control_mode__ = self.node_name.clutchless
        self.start_node_handler(self.node_name.clutchless)
                    
    @pyqtSlot()
    def on_simulation_select(self):
        if self.__mode__ != self.MODE.simulation :
            self.__mode__ = self.MODE.simulation
            if self.__control_mode__ is not None:
                self.thread.kill()
                self.start_node_handler(self.__control_mode__)
    
    @pyqtSlot()
    def on_hardware_select(self):
        if self.__mode__ != self.MODE.hardware :
            self.__mode__ = self.MODE.hardware
            print(self.__control_mode__)
            if self.__control_mode__ is not None:
                self.thread.kill()
                self.start_node_handler(self.__control_mode__)
    
    def start_node_handler(self, name=node_name.clutchNGo):
        msg = QMessageBox()
        msg.setText('running ' + name)
        retval = msg.exec_()
        
        if self.thread is not None:
            self.thread.kill()

        # start teleop
        
#         rospy.Publisher('/dvrk/console/teleop/set_scale', Float32, latch=True, queue_size=1).publish(Float32(0.3))
#         rospy.Publisher('/dvrk/console/teleop/enable', Bool, latch=True, queue_size=1).publish(Bool(True))

        if self.thread_tel is  None and name != self.node_name.clutchless:
            self.thread_tel = self.thread_teleop(self.__mode__)
            self.thread_tel.start()
        
        if name == self.node_name.autocamera:
            self.groupBoxAutocameraParams.setVisible(True)
        else:
            self.groupBoxAutocameraParams.setVisible(False)
            
        if name == self.node_name.clutchNGo :
            self.thread = self.thread_clutchNGo(self.__mode__, self.thread_tel)
            self.thread.start()
        elif name == self.node_name.autocamera:
            self.thread = None
            self.thread = self.thread_autocamera(self.__mode__)
            self.thread.start()
            self.get_autocamera_params()
        elif name == self.node_name.joystick:
            self.thread = self.thread_joystick(self.__mode__)
            self.thread.start()
        elif name == self.node_name.oculus:
            self.thread = self.thread_oculus(self.__mode__)
            self.thread.start()
        elif name == self.node_name.clutchless:
            if self.thread_tel :
                self.thread_tel.kill()
            self.thread = self.thread_clutchless(self.__mode__)
            self.thread.start()
            self.thread.home()
            

def init_yappi():
  OUT_FILE = './yappy.txt'

  import atexit
  import yappi

  print('[YAPPI START]')
  yappi.set_clock_type('wall')
  yappi.start()

  @atexit.register
  def finish_yappi():
    print('[YAPPI STOP]')

    yappi.stop()

    print('[YAPPI WRITE]')

    stats = yappi.get_func_stats()

    for stat_type in ['pstat', 'callgrind', 'ystat']:
      print('writing {}.{}'.format(OUT_FILE, stat_type))
      stats.save('{}.{}'.format(OUT_FILE, stat_type), type=stat_type)

    print('\n[YAPPI FUNC_STATS]')

    print('writing {}.func_stats'.format(OUT_FILE))
    with open('{}.func_stats'.format(OUT_FILE), 'wb') as fh:
      stats.print_all(out=fh)

    print('\n[YAPPI THREAD_STATS]')

    print('writing {}.thread_stats'.format(OUT_FILE))
    tstats = yappi.get_thread_stats()
    with open('{}.thread_stats'.format(OUT_FILE), 'wb') as fh:
      tstats.print_all(out=fh)

    print('[YAPPI OUT]')
            
def main():
#     init_yappi()
    app=QtWidgets.QApplication(sys.argv)
    form = camera_qt_gui()
    form.show()
    #app.exec_()
    sys.exit(app.exec_())
    """
    node_handler = Autocamera_node_handler()
    node_handler.set_mode(node_handler.MODE.hardware)
    node_handler.debug_graphics(False)
    node_handler.spin()
    """
    

if __name__ == "__main__":
    main()
    print('hello')
