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
from PyQt4.QtCore import pyqtSlot, SIGNAL
import threading
from PyQt4.QtCore import QThread
from PyQt4.QtCore import Qt
from std_msgs.msg._Empty import Empty
from geometry_msgs.msg import PoseStamped, Pose
from hrl_geom import pose_converter
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion

import configparser
import camera_control_gui
from PyQt4 import QtGui
from std_msgs.msg._Float32 import Float32
from PyQt4.Qt import QObject
import rosbag
from geometry_msgs.msg._Wrench import Wrench



    
class Teleop_class:
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
        self.scale = 0.5
        self.__enabled__ = False
        self.clutch_active = False
         
        self.last_mtml_pos = None
        self.last_mtml_rot = None
        self.last_mtmr_pos = None
        self.last_mtmr_rot = None
    
        self.last_good_psm1_transform = None
        self.last_good_psm2_transform = None
        
        self.mtml_gripper = None
        self.mtmr_gripper = None
        
        self.last_psm1_jnt = None
        self.last_psm2_jnt = None
        
        self.last_ecm_jnt = None
        
        self.first_mtml_pos = None
        self.first_mtmr_pos = None
        self.first_psm1_pos = None
        self.first_psm2_pos = None
        
        self.T_ecm = None
        self.T_mtml_000 = None
        self.T_mtmr_000 = None
        self.arms_homed = False
        self.__paused__ = False
        
        from autocamera_algorithm import Autocamera
        from visualization_msgs.msg import Marker
        self.autocamera = Autocamera()
        
        
    def __init__nodes(self):
        self.mtml_robot = URDF.from_parameter_server('/dvrk_mtml/robot_description')
        self.mtmr_robot = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
        self.psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
        self.psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        self.ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        
        self.mtml_kin = KDLKinematics(self.mtml_robot, self.mtml_robot.links[0].name, self.mtml_robot.links[-1].name)
        self.mtmr_kin = KDLKinematics(self.mtmr_robot, self.mtmr_robot.links[0].name, self.mtmr_robot.links[-1].name)
        self.psm1_kin = KDLKinematics(self.psm1_robot, self.psm1_robot.links[0].name, self.psm1_robot.links[-1].name)
        self.psm2_kin = KDLKinematics(self.psm2_robot, self.psm2_robot.links[0].name, self.psm2_robot.links[-1].name)
        self.ecm_kin = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[-1].name)
        
    
        # Subscribe to MTMs
        self.sub_mtml = None; self.sub_mtmr = None
        if self.__mode__ == self.MODE.simulation:
            self.sub_mtml = rospy.Subscriber('/dvrk_mtml/joint_states', JointState, self.mtml_cb, queue_size=1, tcp_nodelay=True)
            self.sub_mtmr = rospy.Subscriber('/dvrk_mtmr/joint_states', JointState, self.mtmr_cb, queue_size=1, tcp_nodelay=True)
        elif self.__mode__ == self.MODE.hardware:
            self.sub_mtml = rospy.Subscriber('/dvrk/MTML/state_joint_current', JointState, self.mtml_cb, queue_size=1, tcp_nodelay=True)
            self.sub_mtmr = rospy.Subscriber('/dvrk/MTMR/state_joint_current', JointState, self.mtmr_cb, queue_size=1, tcp_nodelay=True)
        
        # subscribe to head sensor
        self.sub_headsensor_cb = rospy.Subscriber('/dvrk/footpedals/coag', Joy, self.headsensor_cb , queue_size=1, tcp_nodelay=True)
        
        # Subscribe to PSMs
        self.sub_psm1 = None; self.sub_psm2 = None
        if self.__mode__ == self.MODE.simulation:
            self.sub_psm1 = rospy.Subscriber('/dvrk_psm1/joint_states', JointState, self.psm1_cb, queue_size=1, tcp_nodelay=True)
            self.sub_psm2 = rospy.Subscriber('/dvrk_psm2/joint_states', JointState, self.psm2_cb, queue_size=1, tcp_nodelay=True)
            self.sub_ecm = rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.ecm_cb, queue_size=1, tcp_nodelay=True)
        else:
            self.sub_psm1 = rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, self.psm1_cb, queue_size=1, tcp_nodelay=True)
            self.sub_psm2 = rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, self.psm2_cb, queue_size=1, tcp_nodelay=True)
            self.sub_ecm = rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.ecm_cb, queue_size=1, tcp_nodelay=True)
            self.sub_mtml_gripper = rospy.Subscriber('/dvrk/MTML/gripper_position_current', Float32, self.mtml_gripper_cb, queue_size=1, tcp_nodelay=True)
            self.sub_mtmr_gripper = rospy.Subscriber('/dvrk/MTMR/gripper_position_current', Float32, self.mtmr_gripper_cb, queue_size=1, tcp_nodelay=True)
            
        # MTM repositioning clutch
        self.sub_clutch = rospy.Subscriber('/dvrk/footpedals/clutch', Joy, self.clutch_cb, queue_size=1, tcp_nodelay=True)
            
        # Publish to PSMs simulation
        self.pub_psm1 = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=1)
        self.pub_psm2 = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=1)
        
        # Publish to MTMs simulation
        self.pub_mtml = rospy.Publisher('/dvrk_mtml/joint_states_robot', JointState, queue_size=1)
        self.pub_mtmr = rospy.Publisher('/dvrk_mtmr/joint_states_robot', JointState, queue_size=1)
        
        # Translation and orientation lock
        self.lock_mtml_psm2_orientation = rospy.Publisher('/dvrk/MTML_PSM2/lock_rotation', Bool, queue_size=1, latch=True )
        self.lock_mtml_psm2_translation = rospy.Publisher('/dvrk/MTML_PSM2/lock_translation', Bool, queue_size=1, latch=True )
        
        self.lock_mtmr_psm1_orientation = rospy.Publisher('/dvrk/MTMR_PSM1/lock_rotation', Bool, queue_size=1, latch=True )
        self.lock_mtmr_psm1_translation = rospy.Publisher('/dvrk/MTMR_PSM1/lock_translation', Bool, queue_size=1, latch=True )

        # MTML lock orientation
        self.pub_lock_mtml_orientation = rospy.Publisher('/dvrk/MTML/lock_orientation', Quaternion, latch=True, queue_size = 1)
        self.pub_lock_mtmr_orientation = rospy.Publisher('/dvrk/MTMR/lock_orientation', Quaternion, latch=True, queue_size = 1)
        
        self.pub_unlock_mtml_orientation = rospy.Publisher('/dvrk/MTML/unlock_orientation', Empty, latch=True, queue_size = 1)
        self.pub_unlock_mtmr_orientation = rospy.Publisher('/dvrk/MTMR/unlock_orientation', Empty, latch=True, queue_size = 1)
        
        self.pub_mtmr_psm1_teleop = rospy.Publisher('/dvrk/MTMR_PSM1/set_desired_state', String, latch=True, queue_size=1)
        self.pub_mtml_psm2_teleop = rospy.Publisher('/dvrk/MTML_PSM2/set_desired_state', String, latch=True, queue_size=1)
        
        # Wrist Adjustments
        self.mtml_wrist_adjustment = rospy.Publisher('/dvrk/MTML/run_wrist_adjustment', Empty, latch=True)
        self.mtmr_wrist_adjustment = rospy.Publisher('/dvrk/MTMR/run_wrist_adjustment', Empty, latch=True)
        
        # Access psm hardware
        self.hw_psm1 = robot('PSM1')
        self.hw_psm2 = robot('PSM2')
        
        # Access mtm hardware
        self.hw_mtml = robot('MTML')
        self.hw_mtmr = robot('MTMR')
        
        
        
        if self.__mode__ == self.MODE.simulation:
            self.enable_teleop()

    
    def home_arms(self):
        if self.arms_homed :
            return
        # move the psms and mtms to a preferred initial position
        r_psm1 = self.hw_psm1.move_joint_list([-0.491412938252654, 0.004927103573755237, 0.12736308508000002, 0.5969872691004409, -0.09468909433745022, -0.013042400593701237, -0.17428811856527596], interpolate=True)
        r_psm2 = self.hw_psm2.move_joint_list([0.7011160267193464, 0.22136441510677135, 0.12160626119000001, -0.8150648103243462, -0.5017292271386973, 0.06607583378289802, -0.17477815076367886], interpolate=True)
        r_mtml = self.hw_mtml.move_joint_list( [0.044688654936885334, -0.011651854738228436, 0.013282347608626847, -1.536878997367617, 0.7253564294471403, -0.16875639378974283, -0.076976598407095,0.0], interpolate=True)
        r_mtmr = self.hw_mtmr.move_joint_list([0.02710054794953024, -0.018831916576920554, 0.004262673831747664, 1.5243832291804937, 0.8052936686107027, 0.11250426252649523, -0.02960638400272885, 0.0], interpolate=True)
        
        if r_psm1 * r_psm2 * r_mtml * r_mtmr:
            self.arms_homed = True
                
    def shut_down(self):
        self.sub_mtml.unregister()
        self.sub_mtmr.unregister()
        self.sub_psm1.unregister()
        self.sub_psm2.unregister()
        self.sub_ecm.unregister()
        self.sub_headsensor_cb.unregister()
        self.sub_clutch.unregister()
        self.sub_mtml_gripper.unregister()
        self.sub_mtmr_gripper.unregister()
        
        self.pub_psm1.unregister()
        self.pub_psm2.unregister()
        self.pub_mtml.unregister()
        self.pub_mtmr.unregister()
        self.pub_lock_mtml_orientation.unregister()
        self.pub_lock_mtmr_orientation.unregister()
        self.pub_unlock_mtml_orientation.unregister()
        self.pub_unlock_mtmr_orientation.unregister()
        
        self.hw_mtml.unregister()
        self.hw_mtmr.unregister()
        self.hw_psm1.unregister()
        self.hw_psm2.unregister()
        
    
    def set_mode(self, mode):
        self.__mode__ = mode
        
    def spin(self):
        self.__init__nodes()
        rospy.spin()
        
    def pause(self):
        self.__enabled__ = False
    def resume(self):
        self.__enabled__ = True
                
    def enable_teleop(self):
        self.__enabled__ = True
        
        self.first_mtml_pos = self.last_mtml_pos
        self.first_mtmr_pos = self.last_mtmr_pos
        self.first_psm1_pos, _ = self.psm1_kin.FK( self.last_psm1_jnt)
        self.first_psm2_pos, _ = self.psm2_kin.FK( self.last_psm2_jnt)
        
        self.hw_mtml.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
        self.hw_mtml.set_wrench_body_force([0,0,0])
        self.hw_mtml.set_gravity_compensation(True)
        
        self.hw_mtmr.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
        self.hw_mtmr.set_wrench_body_force([0,0,0])
        self.hw_mtmr.set_gravity_compensation(True)
        
        
#         self.align_mtms_to_psms()
        
#         self.hw_mtmr.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
#         self.lock_mtml_psm2_translation.publish(Bool(False))
#         self.lock_mtml_psm2_orientation.publish(Bool(False))
#         self.lock_mtmr_psm1_translation.publish(Bool(False))
#         self.lock_mtmr_psm1_orientation.publish(Bool(False))
        
    def disable_teleop(self):
        self.__enabled__ = False
        self.hw_mtml.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
        self.hw_mtml.set_gravity_compensation(False)
        
        self.hw_mtmr.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
        self.hw_mtmr.set_gravity_compensation(False)
        
#         self.lock_mtml_psm2_translation.publish(Bool(True))
#         self.lock_mtml_psm2_orientation.publish(Bool(True))
#         self.lock_mtmr_psm1_translation.publish(Bool(True))
#         self.lock_mtmr_psm1_orientation.publish(Bool(True))
    
    def rotate(self, axis, angle):
        """
        Returns a rotation matrix and a transformation matrix
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
        t = np.eye(4,4)
        t[0:3, 0:3] = m    
        return m, t

    
    def lock_mtm_orientations(self):
        mtml_pose = self.mtml_kin.forward(self.last_mtml_jnt)
        mtmr_pose = self.mtmr_kin.forward(self.last_mtmr_jnt)
        mtml_quat = pose_converter.PoseConv.to_pos_quat(mtml_pose)
        mtmr_quat = pose_converter.PoseConv.to_pos_quat(mtmr_pose)
#              
        ql = Quaternion(); ql.w, ql.x, ql.y, ql.z = mtml_quat[1]
        qr = Quaternion(); qr.w, qr.x, qr.y, qr.z = mtmr_quat[1]
         
        self.pub_lock_mtml_orientation.publish(ql)
        self.pub_lock_mtmr_orientation.publish(qr)
        
    def clutch_cb(self, msg):
        if msg.buttons[0] == 1 and self.__enabled__:
            if self.clutch_active == False:
                self.clutch_active = True
                self.disable_teleop()
            
            self.lock_mtm_orientations()
            
        elif self.clutch_active == True:
            self.clutch_active = False
            self.enable_teleop()
             
            
            
            
            
    def headsensor_cb(self, msg):
        if msg.buttons[0] == 1:
            self.enable_teleop()            
        else:
            self.disable_teleop()
            
    def ecm_cb(self, msg):
        if self.__mode__ == self.MODE.simulation:
            self.last_ecm_jnt = msg.position[0:2] + msg.position[-2:]
        elif self.__mode__ == self.MODE.hardware:
            self.last_ecm_jnt = msg.position[0:3] + tuple([0])
        
        self.T_ecm = self.ecm_kin.forward(self.last_ecm_jnt)
          
    def psm1_cb(self, msg):
        if self.__mode__ == self.MODE.simulation:
            p = msg.position
            msg.position = [p[0], p[1], p[7], p[8], p[9], p[10], p[11]]
            
        msg.name =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
        self.last_psm1_jnt = msg.position[0:-1]
        if self.__mode__ == self.MODE.hardware:
            self.pub_psm1.publish(msg)
            
            
        
    def psm2_cb(self, msg):
        if self.__mode__ == self.MODE.simulation:
            p = msg.position
            msg.position = [p[0], p[1], p[7], p[8], p[9], p[10], p[11]]
            
        msg.name =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
        self.last_psm2_jnt = msg.position[0:-1]
        if self.__mode__ == self.MODE.hardware:
            self.pub_psm2.publish(msg)
            
    
    def mtml_gripper_cb(self, msg):
        self.mtml_gripper = msg.data
    def mtmr_gripper_cb(self, msg):
        self.mtmr_gripper = msg.data
                
    def mtml_cb(self, msg):
        # Find mtm end effector position and orientation
        self.home_arms()
#         self.align_mtms_to_psms()
        
        if self.last_ecm_jnt == None: return
        if self.__mode__ == self.MODE.simulation:
            msg.position = msg.position[0:2] + msg.position[3:] 
            msg.name = msg.name[0:2] + msg.name[3:]
        else:
            msg.position = msg.position[0:-1]
            msg.name = msg.name[0:-1]
#         msg.position = [.8 * i for i in msg.position]
        
        self.last_mtml_jnt = msg.position
        
        if self.T_mtml_000 == None :
            self.T_mtml_000 = self.mtml_kin.forward(msg.position)

        
        T_mtm = self.mtml_kin.forward(msg.position)
        T = ( self.T_mtml_000**-1) * T_mtm 
        transform = np.matrix( [ [0,-1,0,0], 
                                [0,0,1,0], 
                                [-1,0,0,0], 
                                [0,0,0,1]])
        T = transform * T
        

        
  
        pos = T[0:3,3]
        rot = T[0:3,0:3]
        
        if self.last_mtml_pos == None or self.clutch_active:
            self.first_mtml_pos = pos
            self.last_mtml_pos = pos
            self.last_mtml_rot = rot
            return
        
        self.last_mtml_pos = pos
        self.last_mtml_rot = rot
        
        if self.__enabled__ == False: return
        self.mtml_wrist_adjustment.publish()
        
#         self.autocamera.add_marker(T, 'mtml_delta', scale= [.02,0,0], type=Marker.LINE_LIST, points=[self.first_mtml_pos, pos], frame = "left_wrist_roll_link")
        
        delta = pos - self.first_mtml_pos
        delta = np.insert(delta, 3,1).transpose()
        p0 = np.insert(self.first_mtml_pos, 3,1).reshape(4,1)
        p1 = np.insert(pos, 3,1).transpose().reshape(4,1)
        
        translation = (p1-p0)
        translation = ( self.T_ecm * translation)[0:3]
        
        orientation = self.T_ecm[0:3,0:3] * rot

        T = self.translate_mtml(translation)
        T = self.set_orientation_mtml( orientation, T)
        
        q = list(self.last_psm2_jnt)
        q[5] = 0
        q[4] = 0
        q[3] = 0
        new_psm2_angles = self.psm2_kin.inverse(T, q)
            
        
#         if type(new_psm2_angles) == NoneType:
#             print("Frozen, Translation = " + translation.__str__())
# #             self.first_mtml_pos = self.last_mtml_pos
#             T = self.set_orientation_mtml( self.last_good_psm2_transform[0:3,0:3] ) 
#             new_psm2_angles = self.psm1_kin.inverse(T, q)
            
        if type(new_psm2_angles) == NoneType:
            self.hw_mtml.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
            self.hw_mtml.set_gravity_compensation(False)
            self.pub_unlock_mtml_orientation.publish()
            return
        else:
            self.hw_mtml.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
            self.hw_mtml.set_wrench_body_force([0, 0, 0])
            self.hw_mtml.set_gravity_compensation(True)
            
        self.last_good_psm2_transform = T
        
        if self.__mode__ == self.MODE.hardware:
            gripper = (self.mtml_gripper-.4) * 1.4/.6
            new_psm2_angles = np.append(new_psm2_angles, gripper)
            
        if self.__mode__ == self.MODE.hardware:
            self.hw_psm2.move_joint_list( new_psm2_angles.tolist(), range(0,len(new_psm2_angles)), interpolate=False)
        
        if self.__mode__ == self.MODE.simulation:
            msg = JointState()
            msg.position = new_psm2_angles.tolist()
            msg.name =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
            self.pub_psm2.publish(msg)
            
        
        
        
    
    def mtmr_cb(self, msg):
        # Find mtm end effector position and orientation
        if self.last_ecm_jnt == None: return
        
        if self.__mode__ == self.MODE.simulation:
            msg.position = msg.position[0:2] + msg.position[3:] 
            msg.name = msg.name[0:2] + msg.name[3:]
        else:
            msg.position = msg.position[0:-1]
            msg.name = msg.name[0:-1]

#         msg.position = [.8 * i for i in msg.position]
        self.last_mtmr_jnt = msg.position
        
        if self.T_mtmr_000 == None :
            self.T_mtmr_000 = self.T_mtml_000
                        
        T_mtm = self.mtmr_kin.forward(msg.position)
        
        T = ( self.T_mtmr_000**-1) * T_mtm 
        transform = np.matrix( [ [0,-1,0,0], 
                                [0,0,1,0], 
                                [-1,0,0,0], 
                                [0,0,0,1]])
        T = transform * T
        
        rot = T[0:3,0:3]
        pos = T[0:3,3]
        
        
        if self.last_mtmr_pos == None  or self.clutch_active:
            self.first_mtmr_pos = pos
            self.last_mtmr_pos = pos
            self.last_mtmr_rot = rot
            return
        
        self.last_mtmr_pos = pos
        self.last_mtmr_rot = rot
        
        if self.__enabled__ == False: return
        
        self.mtmr_wrist_adjustment.publish()
        
        delta = pos - self.first_mtmr_pos
        delta = np.insert(delta, 3,1).transpose()
        p0 = np.insert(self.first_mtmr_pos, 3,1).reshape(4,1)
        p1 = np.insert(pos, 3,1).transpose().reshape(4,1)
        
        translation = (p1-p0)
        translation = ( self.T_ecm * translation)[0:3]
        
        orientation = self.T_ecm[0:3,0:3] * rot
        
        T = self.translate_mtmr(translation)
        T = self.set_orientation_mtmr(orientation, T)
        
        q = list(self.last_psm1_jnt)
        q[5] = 0
        q[4] = 0
        q[3] = 0
        new_psm1_angles = self.psm1_kin.inverse(T, q)
        
#         if type(new_psm1_angles) == NoneType:
#             T[0:3, 0:3] = self.last_good_psm1_transform[0:3,0:3] 
#             new_psm1_angles = self.psm1_kin.inverse(T, q)
            
        if type(new_psm1_angles) == NoneType:
            self.hw_mtmr.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
            self.hw_mtmr.set_gravity_compensation(False)
            self.pub_unlock_mtmr_orientation.publish()
            return
        else:
            self.hw_mtmr.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
            self.hw_mtmr.set_wrench_body_force([0, 0, 0])
            self.hw_mtmr.set_gravity_compensation(True)
        
#         self.last_good_psm1_transform = T
        
        if self.__mode__ == self.MODE.hardware:
            gripper = (self.mtmr_gripper-.4) * 1.2/.4
            new_psm1_angles = np.append(new_psm1_angles, gripper)
                
        if self.__mode__ == self.MODE.hardware:
            self.hw_psm1.move_joint_list( new_psm1_angles.tolist(), range(0,len(new_psm1_angles)), interpolate=False)
        
        if self.__mode__ == self.MODE.simulation:
            msg = JointState()
            msg.position = new_psm1_angles.tolist()
            msg.name =  ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
            self.pub_psm1.publish(msg)
            
        
        
    
    def align_mtms_to_psms(self):
        T_psm1 = self.psm1_kin.forward(self.last_psm1_jnt)
        T_psm2 = self.psm2_kin.forward(self.last_psm2_jnt)
        
        T_mtml = self.mtml_kin.forward(self.last_mtml_jnt)
        T_mtmr = self.mtmr_kin.forward(self.last_mtmr_jnt)
        T_mtml_000 = self.mtml_kin.forward([0,0,0,0,0,0,0])
        
        T_mtml[0:3,0:3] = (((self.T_ecm * (T_mtml_000 ** -1)) ** -1) * T_psm2)[0:3,0:3]
        T_mtmr[0:3, 0:3] = T_psm1[0:3, 0:3]
        
        jnt_mtml = self.mtml_kin.inverse(T_mtml)
        jnt_mtmr = self.mtmr_kin.inverse(T_mtmr)
        print('aligning')
        if self.__mode__ == self.MODE.hardware:
            self.hw_mtml.move_joint_list( jnt_mtml.tolist(), range(0, len(jnt_mtml)), interpolate=True)
            self.hw_mtmr.move_joint_list( jnt_mtmr.tolist(), range(0, len(jnt_mtmr)), interpolate=True)
            
        
        
    def translate_mtml(self, translation, T=None): # translate a psm arm
        if self.__enabled__ == False: return
        
        translation = translation * self.scale
        psm2_pos = self.first_psm2_pos#self.psm2_kin.FK(self.last_psm2_jnt)
        if T==None:
            T = self.psm2_kin.forward(self.last_psm2_jnt)
        new_psm2_pos = psm2_pos + translation
        T[0:3, 3] = new_psm2_pos
#         self.autocamera.add_marker(T, 'psm2_delta', color = [1,1,0], scale= [.02,0,0], type=Marker.LINE_LIST, points=[psm2_pos,new_psm2_pos], frame="world")
        return T
    
    def translate_mtmr(self, translation, T=None): # translate a psm arm
        if self.__enabled__ == False: return
        
        translation = translation * self.scale
        psm1_pos = self.first_psm1_pos #self.psm1_kin.FK(self.last_psm1_jnt)
        
        if T==None:
            T = self.psm1_kin.forward(self.last_psm1_jnt)
            
        new_psm1_pos = psm1_pos + translation
        T[0:3, 3] = new_psm1_pos
        
        return T
        
    
    def set_orientation_mtml(self,orientation, T=None): # align a psm arm to mtm
        if self.__enabled__ == False: return
        
        if T==None:
            T = self.psm2_kin.forward(self.last_psm2_jnt)
        T[0:3,0:3] = orientation
        return T
        
    def set_orientation_mtmr(self,orientation, T = None): # align a psm arm to mtm
        if self.__enabled__ == False: return
        if T == None:
            T = self.psm1_kin.forward(self.last_psm1_jnt)
        T[0:3,0:3] = orientation
        return T
            
        
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
        dir = '{}/'.format(recording_dir)+bag_name +'/'; dir = dir.__str__()
        if not os.path.exists(dir):
            os.system( ('mkdir -p '+ dir ).__str__())
        
        self.bag_sim = rosbag.Bag(dir + bag_name + '_sim.bag', 'w')
        self.bag_hw = rosbag.Bag(dir + bag_name + '_hw.bag', 'w')
        self.arm_names = arm_names
        # The topics we want to record
#         self.topics = '/dvrk/{}/state_joint_current'.format(ARM_NAME)
        self.topics = { arm_name:'/dvrk/{}/state_joint_current'.format(arm_name) for arm_name in self.arm_names}
        self.out_topics_hw = {arm_name : '/dvrk/{}/set_position_joint'.format(arm_name) for arm_name in self.arm_names}
        self.out_topics_sim = {arm_name : '/dvrk_{}/joint_states_robot'.format(arm_name.lower()) for arm_name in self.arm_names}
        
#         self.out_topics = self.out_topics_sim
        self.out_topics = self.out_topics_hw
        # We have to initialize a ros node if we want to subsribe or publish messages
#         rospy.init_node('rosbag_test_node')
        
        for arm_name in self.arm_names:
            exec("self.sub_{} = rospy.Subscriber('{}', JointState, self.cb_{})".format(arm_name, self.topics[arm_name], arm_name))
            rospy.timer.sleep(.1)
    def set_mode(self, mode):
        self.__mode__ = mode
    
    def shutdown(self):
        # unregister all subscribers
        for arm_name in self.arm_names:
            eval("self.sub_{}.unregister()".format(arm_name))
            rospy.timer.sleep(.1)
            
        self.bag_hw.close()
        self.bag_sim.close()
        
        
    def spin(self):
        rospy.spin()
        
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
    def cb(self,arm_name, msg):
        try:
            self.bag_sim.write(self.out_topics_sim[arm_name], msg) # record the msg
            self.bag_hw.write(self.out_topics_hw[arm_name], msg) # record the msg
        except Exception:
            print("there was an error") 
    
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
        
        # Get zoom parameters from config file
        self.config = configparser.ConfigParser()
        self.config_file = '../config/autocamera.conf'
        self.config.read(self.config_file)
        
        inner = self.config.getfloat('ZOOM', 'inner_zone')
        outer = self.config.getfloat('ZOOM', 'dead_zone')
        
        self.set_autocamera_params(inner, outer)
        
        self.initialize_psms_initialized = 30
        self.__DEBUG_GRAPHICS__ = False
        
        
    def __init_nodes__(self):
        self.hw_ecm = robot('ECM')
        self.hw_psm1 = robot('PSM1')
        self.hw_psm2 = robot('PSM2')
            
        #rospy.init_node('autocamera_node')
        
        self.logerror("start", debug=True)
        
        # Publishers to the simulation
        self.pub_ecm = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=1, tcp_nodelay=True)
        self.pub_psm1 = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=1, tcp_nodelay=True)
        self.pub_psm2 = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=1, tcp_nodelay=True)
        
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
            
            
        elif self.__AUTOCAMERA_MODE__ == self.MODE.simulation:
            # Get the joint angles from the simulation
            self.sub_psm1_sim = rospy.Subscriber('/dvrk_psm1/joint_states', JointState, self.add_psm1_jnt, queue_size=1, tcp_nodelay=True)
            self.sub_psm2_sim = rospy.Subscriber('/dvrk_psm2/joint_states', JointState, self.add_psm2_jnt, queue_size=1, tcp_nodelay=True)
            
            # If hardware is connected, subscribe to it and set the psm joint angles in the simulation from the hardware
#             self.sub_psm1_hw = rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, self.add_psm1_jnt_from_hw)
#             self.sub_psm2_hw = rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, self.add_psm2_jnt_from_hw)
            
            
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
                self.sub_ecm_sim.unregister()
            self.sub_psm1_hw.unregister()
            self.sub_psm2_hw.unregister()
            self.sub_caminfo.unregister()
            
            self.pub_image_left.unregister()
            self.pub_image_right.unregister()
            self.pub_ecm.unregister()
            self.pub_psm1.unregister()
            self.pub_psm2.unregister()
            
            self.hw_ecm.unregister()
            self.hw_psm1.unregister()
            self.hw_psm2.unregister()
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
        
        self.config.set('ZOOM', 'inner_zone', inner_zone.__str__())
        self.config.set('ZOOM', 'dead_zone', dead_zone.__str__())
        
        with open(self.config_file,'w') as cfile:
            self.config.write(cfile)
        
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
        
        self.pub_ecm.publish(new_ecm_msg)
        
        
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
        
        self.hw_psm1.move_joint_list(msg.position, interpolate=False)
    
    def move_psm2(self, msg):
        jaw = msg.position[-3]
        msg = self.autocamera.extract_positions(msg, 'psm2')
        
        msg.name = msg.name + ['jaw']
        msg.position = msg.position + [jaw]
        
        self.hw_psm2.move_joint_list(msg.position, interpolate=False)
    
    # ecm callback    
    def add_ecm_jnt(self, msg):
        if self.camera_clutch_pressed == False and msg != None:
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
        if self.camera_clutch_pressed == False and msg != None:
            if self.__AUTOCAMERA_MODE__ == self.MODE.simulation:
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.pub_psm1.publish(msg)

    def add_psm2_jnt_from_hw(self, msg):
        if self.camera_clutch_pressed == False and msg != None:
            if self.__AUTOCAMERA_MODE__ == self.MODE.simulation:
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.pub_psm2.publish(msg)
                
    # psm1 callback    
    def add_psm1_jnt(self, msg):
        if self.camera_clutch_pressed == False and msg != None:
            # We need to set the names, otherwise the simulation won't move
            if self.__AUTOCAMERA_MODE__ == self.MODE.hardware:
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.pub_psm1.publish(msg)
            self.add_jnt('psm1', msg)
                
         
    # psm2 callback    
    def add_psm2_jnt(self, msg):
        if self.camera_clutch_pressed == False and msg != None:
            if self.__AUTOCAMERA_MODE__ == self.MODE.hardware :
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                self.pub_psm2.publish(msg)
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
                
                self.pub_ecm.publish(jnt_msg)
                
                jnt_msg.position = [ round(i,4) for i in jnt_msg.position]
                if len(jnt_msg.position) != 4 or len(jnt_msg.name) != 4 :
                    return
                #return # stop here until we co-register the arms
                if self.__AUTOCAMERA_MODE__ == self.MODE.hardware:
                    pos = jnt_msg.position
#                     if self.first_run:
#                         result = self.hw_ecm.move_joint_list(pos, index=[0,1,2,3], interpolate=self.first_run)
#                     else:
#                     pos = jnt_msg.position[0:2] + [jnt_msg.position[3]]
                    result = self.hw_ecm.move_joint_list(pos, index=[0,1,2,3], interpolate=self.first_run)
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
            self.pub_psm1.publish(msg)
            msg.position = [-0.84 , -0.53, 0.10, 0.00, 0.00, 0.00, 0.00]
            self.pub_psm2.publish(msg)
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
        
        
    def __init__(self, teleop_thread, mode = MODE.simulation):        
        self.__mode__ = mode
        self.teleop_thread = teleop_thread
        
        self.camera_clutch_pressed = False
        self.movement_scale = 1.5 
        self.joint_angles = [0,0,0,0]
        self.center = [0,0,0,0]
        
        self.flag = True
        
        self.mtml_pos = [0,0,0]
        self.mtml_joint_angles = [0,0,0,0,0,0,0]
        
        self.mtmr_pos = [0,0,0]
        self.mtmr_joint_angles = [0,0,0,0,0,0,0]
        
        self.mtmr_starting_point =  [0,0,0,0,0,0,0]
        
    def __init_nodes__(self):
#         rospy.init_node('ecm_clutch_control')
        
        self.hw_ecm = robot("ECM")
        self.hw_mtmr = robot("MTMR")
        self.hw_mtml = robot("MTML")
        
        self.pub_mtml_hw = rospy.Publisher('/dvrk/MTML/set_position_joint', JointState, queue_size=1)
        self.pub_mtmr_hw = rospy.Publisher('/dvrk/MTMR/set_position_joint', JointState, queue_size=1)
        
        self.pub_ecm_sim = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=10)
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
        
        self.hw_mtml_orientation = mtm('MTML')
        self.hw_mtmr_orientation = mtm('MTMR')
        
        # MTML lock orientation
#         self.pub_lock_mtml_orientation = rospy.Publisher('/dvrk/MTML/lock_orientation', Quaternion, latch=True, queue_size = 1)
#         self.pub_lock_mtmr_orientation = rospy.Publisher('/dvrk/MTMR/lock_orientation', Quaternion, latch=True, queue_size = 1)
        
        
        self.sub_ecm_cb = None
        if self.__mode__ == self.MODE.simulation:
            self.sub_ecm_cb = rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.ecm_cb)
        elif self.__mode__ == self.MODE.hardware:
            self.sub_ecm_cb = rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.ecm_cb)
#             self.hw_ecm.home()
            self.hw_ecm.move_joint_list([0.0,0.0,0.0,0.0], interpolate=True)
        
        self.camera_clutch_pressed = False
        self.head_sensor_pressed = False
        self.sub_camera_clutch_cb = rospy.Subscriber('/dvrk/footpedals/camera_minus', Joy, self.camera_clutch_cb )
        self.sub_headsensor_cb = rospy.Subscriber('/dvrk/footpedals/coag', Joy, self.headsensor_cb )
        self.mtml_starting_point = None
        
        self.sub_mtml_cart_cb = rospy.Subscriber('/dvrk/MTML/position_cartesian_local_current', PoseStamped, self.mtml_cb)
        self.sub_mtml_joint_cb = rospy.Subscriber('/dvrk/MTML/state_joint_current', JointState, self.mtml_joint_angles_cb)
        
        self.sub_mtmr_cart_cb = rospy.Subscriber('/dvrk/MTMR/position_cartesian_local_current', PoseStamped, self.mtmr_cb)
        self.sub_mtmr_joint_cb = rospy.Subscriber('/dvrk/MTMR/state_joint_current', JointState, self.mtmr_joint_angles_cb)
        
        self.sub_psm1_joint_cb = rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, self.psm1_joint_angles_cb)
        self.sub_psm2_joint_cb = rospy.Subscriber('/dvrk/PSM2/state_joint_current', JointState, self.psm2_joint_angles_cb)
        
        self.T_mtml_pos_init = self.mtml_kin.forward([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.T_mtmr_pos_init = self.T_mtml_pos_init
#         self.hw_mtml_orientation.lock_orientation_as_is()
#         self.hw_mtml_orientation.unlock_orientation()
    
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
            
            self.hw_mtml.unregister()
            self.hw_mtmr.unregister()
            self.hw_mtml_orientation.unregister()
            self.hw_mtmr_orientation.unregister()
            
            self.pub_mtml_hw.unregister()
            self.pub_mtmr_hw.unregister()
            self.pub_ecm_sim.unregister()
#             self.pub_lock_mtml_orientation.unregister()
#             self.pub_lock_mtmr_orientation.unregister()
            
            print( "Shutting down " + self.__class__.__name__)
            
        except(e):
            print("couldn't unregister all the topics")
            pass
#         rospy.signal_shutdown('shutting down ClutchControl')
        
    def set_scale(self, scale):
        self.movement_scale = scale
            
    def set_mode(self, mode):
        """ Values:
            MODE.simulation
            MODE.hardware
        """
        self.__mode__ = mode
        
            
    def spin(self):
        self.__init_nodes__()
        rospy.spin()

    def mtml_joint_angles_cb(self, msg):
        self.mtml_joint_angles = list(msg.position)[0:-1]
        
        T_mtm = self.mtml_kin.forward(self.mtml_joint_angles)
        T = ( self.T_mtml_pos_init **-1) * T_mtm 
        transform = np.matrix( [ [0,-1,0,0], 
                                [0,0,1,0], 
                                [-1,0,0,0], 
                                [0,0,0,1]])
        T = transform * T
        pos = T[0:3,3]
        if self.camera_clutch_pressed:
            if type(self.mtml_starting_point) == NoneType:
                self.mtml_starting_point = pos
            
            # we may multiply the current_position and mtml_pos_before_clutch by some transformation matrix so
            # the hand controllers feel more intuitive
            
            movement_vector = pos-self.mtml_pos_before_clutch
            self.move_mtm_centerpoints()
#             print("movement_vector = {}, {}, {}".format(movement_vector[0], movement_vector[1], movement_vector[2]))
            self.mtml_pos = pos
            self.ecm_pan_tilt(movement_vector)
        else:
#             self.center = self.joint_angles
            try:
                self.mtml_pos_before_clutch = pos
            except:
                pass
            
    
    def mtmr_joint_angles_cb(self, msg):
        self.mtmr_joint_angles = list(msg.position)[0:-1]
        
        T_mtm = self.mtmr_kin.forward(self.mtml_joint_angles)
        T = ( self.T_mtmr_pos_init **-1) * T_mtm 
        transform = np.matrix( [ [0,-1,0,0], 
                                [0,0,1,0], 
                                [-1,0,0,0], 
                                [0,0,0,1]])
        T = transform * T
        
        pos = T[0:3,3]
        if self.camera_clutch_pressed:
            if type(self.mtmr_starting_point) == NoneType:
                self.mtmr_starting_point = pos
            
            movement_vector = pos-self.mtmr_pos_before_clutch
            self.mtmr_pos = pos
        else:
            try:
                self.mtmr_pos_before_clutch = pos
            except:
                pass
            
    def psm1_joint_angles_cb(self,msg):
        self.psm1_joint_angles = list(msg.position)
    
    def psm2_joint_angles_cb(self,msg):
        self.psm2_joint_angles = list(msg.position)
    
    def mtml_cb(self, msg):
        return
#         msg.pose.position.x
        if self.camera_clutch_pressed:
            if type(self.mtml_starting_point) == NoneType:
                self.mtml_starting_point = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            
            # we may multiply the current_position and mtml_pos_before_clutch by some transformation matrix so
            # the hand controllers feel more intuitive
            
            current_position = self.mtml_kin.forward(list(self.mtml_joint_angles)[0:-1])[0:3,3] 
            movement_vector = current_position-self.mtml_pos_before_clutch
            self.move_mtm_centerpoints()
#             print("movement_vector = {}, {}, {}".format(movement_vector[0], movement_vector[1], movement_vector[2]))
            self.mtml_pos = current_position
        else:
#             self.center = self.joint_angles
            try:
                self.mtml_pos_before_clutch = self.mtml_kin.forward(list(self.mtml_joint_angles)[0:-1])[0:3,3]
            except:
                pass
    
    def mtmr_cb(self, msg):
        return
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
            self.hw_mtmr.move_joint_list(mtmr_joint_angles[0:3], [0,1,2], interpolate=False)
            
#             self.move_mtm_out_of_the_way()
             
        
    def enable_teleop(self):
        self.hw_mtml.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
        self.hw_mtml.set_wrench_body_force([0,0,0])
        self.hw_mtml.set_gravity_compensation(True)
        
        self.hw_mtmr.dvrk_set_state('DVRK_EFFORT_CARTESIAN')
        self.hw_mtmr.set_wrench_body_force([0,0,0])
        self.hw_mtmr.set_gravity_compensation(True)
        
    def disable_teleop(self):
        self.hw_mtml.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
        self.hw_mtml.set_gravity_compensation(False)
        
        self.hw_mtmr.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
        self.hw_mtmr.set_gravity_compensation(False)
                
    def headsensor_cb(self, msg):
        if msg.buttons[0] == 1:
            self.head_sensor_pressed = True
#             self.enable_teleop()
        else:
            self.head_sensor_pressed = False
#             self.hw_mtml.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
#             self.hw_mtmr.dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')
                        
    def camera_clutch_cb(self, msg):
        if msg.buttons[0] == 1 and self.head_sensor_pressed:
            if self.camera_clutch_pressed == False:
                self.camera_clutch_pressed = True
                self.mtml_starting_point = None
                self.teleop_thread.pause()
                self.hw_mtml_orientation.lock_orientation_as_is()
                self.hw_mtmr_orientation.lock_orientation_as_is()
#                 self.teleop_thread.lock_mtm_orientations()
            
        elif self.camera_clutch_pressed == True:
            self.camera_clutch_pressed = False
            if self.head_sensor_pressed:
                self.teleop_thread.resume()
                self.hw_mtml_orientation.unlock_orientation()
                self.hw_mtmr_orientation.unlock_orientation()
            
            
                            
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
#         print("self.center is : " + self.center.__str__())  
#         print("ecm joint angles are : " + msg.position.__str__())
#         print("self.joint_angles is : " + self.joint_angles.__str__())
    
    def ecm_pan_tilt(self, movement_vector):
        q = []
        q = self.joint_angles
        movement_vector = [float(i) for i in movement_vector]
        if q:
            q = list(q)
            q[0] = self.center[0] + movement_vector[0] * self.movement_scale # pitch
            q[1] = self.center[1] - movement_vector[1] * self.movement_scale # yaw
            q[2] = self.center[2] - movement_vector[2] *.2 * self.movement_scale # insertion
            
            q = [round(i,4) for i in q]
#             q = [i+j for i,j in zip(self.center, q)]
            self.move_ecm(q)
    
    # move ecm based on joint angles either in simulation or hardware
    def move_ecm(self, joint_angles):
        if self.__mode__ == self.MODE.simulation:
            msg = JointState()
            msg.position = joint_angles
            msg.name = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
            self.pub_ecm_sim.publish(msg)            
        elif self.__mode__ == self.MODE.hardware:
            self.hw_ecm.move_joint_list(joint_angles, interpolate=False)
    
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
        
    def __init_nodes__(self):
        self.hw_ecm = robot("ECM")
        self.pub_ecm_sim = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=10)
        self.ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        self.ecm_kin = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[-1].name)
        
        self.sub_ecm = None
        if self.__mode__ == self.MODE.simulation:
            self.sub_ecm = rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.ecm_cb)
        elif self.__mode__ == self.MODE.hardware:
            self.sub_ecm = rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.ecm_cb)
#             self.hw_ecm.home()
            self.hw_ecm.move_joint_list([0.0,0.0,0.0,0.0], interpolate=True)
            
        self.sub_joy = rospy.Subscriber('/joy', msg.Joy, self.on_joystick_change_cb)
        
    def shutdown(self):
        print('shutting down joystick control')
        try:
            self.sub_ecm.unregister()
            self.sub_joy.unregister()
            self.pub_ecm_sim.unregister()
            self.hw_ecm.unregister()
            
            print( "Shutting down " + self.__class__.__name__)
        except e:
            print("couldn't unregister all the topics")

    def set_mode(self, mode):
        self.__mode__ = mode
                
    def spin(self):
        self.__init_nodes__()
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
        if len(message.buttons) == 0:
            message.buttons = [0]
        if len(message.axes) < 4:
            message.axes = tuple( [a for a in message.axes] + [1])
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
            self.pub_ecm_sim.publish(msg)            
        elif self.__mode__ == self.MODE.hardware:
            if index == []:
                index = range(0,len(joint_angles))
            self.hw_ecm.move_joint_list(joint_angles, index, interpolate=False)
    
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
            
class Oculus:
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
        
    def __init_nodes__(self):
        self.hw_ecm = robot("ECM")
        self.pub_ecm_sim = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=10)
        self.ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        self.ecm_kin = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[-1].name)
        
        self.sub_ecm = None
        if self.__mode__ == self.MODE.simulation:
            self.sub_ecm = rospy.Subscriber('/dvrk_ecm/joint_states', JointState, self.ecm_cb)
        elif self.__mode__ == self.MODE.hardware:
            self.sub_ecm = rospy.Subscriber('/dvrk/ECM/state_joint_current', JointState, self.ecm_cb)
#             self.hw_ecm.home()
            self.hw_ecm.move_joint_list([0.0,0.0,0.0,0.0], interpolate=True)
            
        self.sub_oculus = rospy.Subscriber('/oculus', msg.JointState, self.on_oculus_cb)
        
    def shutdown(self):
        print('shutting down oculus control')
        try:
            self.sub_ecm.unregister()
            self.sub_oculus.unregister()
            self.pub_ecm_sim.unregister()
            
            print( "Shutting down " + self.__class__.__name__)
        except e:
            print("couldn't unregister all the topics")

    def set_mode(self, mode):
        self.__mode__ = mode
                
    def spin(self):
        self.__init_nodes__()
        self.__spin__()
    def __spin__(self):
        rospy.spin()
        
    def on_oculus_cb(self, message):
        if self.__mode__ == self.MODE.simulation:
            message.name = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
            self.pub_ecm_sim.publish(message)
        elif self.__mode__ == self.MODE.hardware:
            print(message.__str__())
            self.hw_ecm.move_joint_list( list(message.position), interpolate=False)
            
    def ecm_cb(self, message):
        pass
class camera_qt_gui(QtGui.QMainWindow, camera_control_gui.Ui_Dialog):
    
    class node_name:
        clutchNGo = 'clutch_control'
        autocamera = 'Autocamera'
        joystick = 'joystick'
        teleop = 'teleop'
        oculus = 'oculus'
        
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
            self.node_handler = Teleop_class()
            print('\nRunning {} in {}\n'.format("Teleop",self.__mode__))
            self.node_handler.set_mode(self.__mode__)
            self.node_handler.spin()
        
        def kill(self):
            if self.node_handler != None:
                self.node_handler.shutdown()
                self.quit()
                
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
            if self.node_handler != None:
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
            if self.node_handler != None:
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
    
    class thread_oculus(QThread):
        def __init__(self, mode):
            super(QThread, self).__init__()
            self.node_handler = None
            self.__mode__ = mode
        def run(self):
            self.node_handler = Oculus()
            print('\nRunning {} in {}\n'.format("Oculus Control",self.__mode__))
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
            return
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
                
                self.pub_psm1 = rospy.Publisher('/dvrk/PSM1/set_base_frame', Pose, queue_size=10)
                self.pub_psm2 = rospy.Publisher('/dvrk/PSM2/set_base_frame', Pose, queue_size=10)
            
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
                    self.pub_psm1.publish(psm1_message)
                    self.pub_psm2.publish(psm2_message)
    #             ecm_message = pose_converter.PoseConv.to_pose_msg(ecm_base_frame)
            else:
                self.sub_ecm.unregister()
                self.pub_psm1.unregister()
                self.pub_psm2.unregister()
            
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
            
    def __init__(self, parent=None):
        super(camera_qt_gui, self).__init__(parent)
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
        self.pushButtonExit.clicked.connect(self.exit_program)
        
        self.radioButtonTeleop.clicked.connect(self.on_teleop_select)
        self.radioButtonAutocamera.clicked.connect(self.on_autocamera_select)
        self.radioButtonClutchAndMove.clicked.connect(self.on_clutchNGo_select)
        self.radioButtonJoystick.clicked.connect(self.on_joystick_select)
        self.radioButtonOculus.clicked.connect(self.on_oculus_select)
        
        self.horizontalSliderInnerzone.valueChanged[int].connect(self.horizontalSliderInnerzoneCb)
        self.horizontalSliderDeadzone.valueChanged[int].connect(self.horizontalSliderDeadzoneCb)
        
        self.radioButtonSimulation.setChecked(True)
        self.radioButtonSimulation.clicked.connect(self.on_simulation_select)
        self.radioButtonHardware.clicked.connect(self.on_hardware_select)
        
        self.groupBoxAutocameraParams.setVisible(False)
        
        self.pushButtonHome.setEnabled(False)
        self.pushButtonPowerOff.setEnabled(False)
        self.pushButtonRecord.setEnabled(False)
        self.textEditFilename.setEnabled(False)
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
        except Exception:
            pass
    
    @pyqtSlot()
    def on_record(self):
        if self.recording == False:
            file_name = self.textEditFilename.toPlainText()
            if os.path.exists('{}/'.format(self.recording_dir)+file_name):
                self.labelFilename.setText('File already exists')
                self.labelFilename.setStyleSheet("color: red")
                return
            else:
                self.labelFilename.setText('')
            self.recording = True
            self.textEditFilename.setEnabled(False)
            self.pushButtonRecord.setStyleSheet("background-color: red")
            self.pushButtonRecord.setText('Stop Recording')
            arm_names = ['MTML', 'MTMR', 'PSM1', 'PSM2', 'ECM']
            self.bag_writer = self.thread_bag_writer(arm_names, file_name, recording_dir=self.recording_dir, mode=self.MODE.hardware)
            self.bag_writer.start()
        else:
            self.recording = False
            self.pushButtonRecord.setStyleSheet("background-color: ")
            self.pushButtonRecord.setText('Record')
            self.bag_writer.kill()
            self.textEditFilename.setEnabled(True)
        
    
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
        while self.thread.node_handler == None:
                pass
            
        dead_zone, inner_zone = self.thread.node_handler.get_autocamera_params()
        self.horizontalSliderDeadzone.setValue(dead_zone*100)
        self.horizontalSliderInnerzone.setValue(inner_zone*100)
        
    @pyqtSlot()
    def home(self):
        self.groupBoxOperationMode.setEnabled(True)
        self.groupBoxCameraControlMethod.setEnabled(True)
        self.pushButtonRecord.setEnabled(True)
        self.textEditFilename.setEnabled(True)
        
        self.homing_thread = self.thread_home_arms()
        self.homing_thread.start()

    @pyqtSlot()
    def power_on(self):
        self.pushButtonHome.setEnabled(True)
        self.pushButtonExit.setEnabled(False)
        self.pushButtonPowerOff.setEnabled(True)
        self.pushButtonPowerOn.setEnabled(False)

        rospy.Publisher('/dvrk/console/home', Empty, latch=True, queue_size=1).publish()
    
    @pyqtSlot()
    def power_off(self):
        self.pushButtonHome.setEnabled(False)
        self.pushButtonExit.setEnabled(True)
        self.pushButtonPowerOff.setEnabled(False)
        self.pushButtonPowerOn.setEnabled(True)
        self.groupBoxOperationMode.setEnabled(False)
        self.groupBoxCameraControlMethod.setEnabled(False)
        
        rospy.Publisher('/dvrk/console/power_off', Empty, latch=True, queue_size=1).publish(Empty())
                
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

        # start teleop
        
#         rospy.Publisher('/dvrk/console/teleop/set_scale', Float32, latch=True, queue_size=1).publish(Float32(0.3))
#         rospy.Publisher('/dvrk/console/teleop/enable', Bool, latch=True, queue_size=1).publish(Bool(True))

        if self.thread_tel == None:
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
    app = QtGui.QApplication(sys.argv)
    form = camera_qt_gui()
    form.show()
    app.exec_()
    """
    node_handler = Autocamera_node_handler()
    node_handler.set_mode(node_handler.MODE.hardware)
    node_handler.debug_graphics(False)
    node_handler.spin()
    """
    

if __name__ == "__main__":
    main()
    print('hello')
