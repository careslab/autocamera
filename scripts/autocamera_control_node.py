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
import configparser

import pexpect
import time
from time import strptime

class Autocamera_node_handler:
    # move the actual ecm with sliders?
    __MOVE_ECM_WITH_SLIDERS__ = False
    class MODE:
        simulation = "SIMULATION"
        hardware = "HARDWARE"
        sliders = "SLIDERS"    
    
    DEBUG = True
    
    def __init__(self):

        rospy.init_node('autocamera_control_node')
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
            tool1 = self.autocamera.zoom_level_positions[tool1_name[camera_name]]; 
            tool1 = tuple(int(i) for i in tool1)

            tool2 = self.autocamera.zoom_level_positions[tool2_name[camera_name]];
            tool2 = tuple(int(i) for i in tool2)
             
            toolm = self.autocamera.zoom_level_positions[toolm_name[camera_name]]; 
            toolm = tuple(int(i) for i in toolm)
            
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
                r = self.hw_ecm.move_joint_list(temp, interpolate=self.first_run)
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

        #Dont return if in simulation since there is no need for head sensor and clutch
        if self.__AUTOCAMERA_MODE__ != self.MODE.simulation:
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

def main():
    #class thread_autocamera(QThread):
    #def __init__(self, mode):
    #super(QThread, self).__init__()
    #self.node_handler = None
    #self.__mode__ = mode

    #def run(self):
    node_handler = Autocamera_node_handler()
    __mode__ = node_handler.MODE.hardware
    print('\nRunning {} in {}\n'.format("Autocamera",__mode__))
    node_handler.set_mode(__mode__)
    node_handler.debug_graphics(True)
    node_handler.spin()

    

if __name__ == "__main__":
    print("Starting..")
    main()