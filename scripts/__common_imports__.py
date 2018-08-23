import os
import tf
import cv2
import math
import time
import rospy
import rosbag
import cv_bridge
import numpy as np
import image_geometry

from arm import arm as robot
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg._CompressedImage import CompressedImage
from sensor_msgs.msg import Joy
from std_msgs.msg._Empty import Empty
from std_msgs.msg._Float32 import Float32
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg._Wrench import Wrench
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import Marker
from visualization_msgs.msg._Marker import Marker
from types import NoneType
from hrl_geom import pose_converter
from hrl_geom.pose_converter import PoseConv
from math import acos, atan2, cos, pi, sin
from numpy import array, cross, dot, float64, hypot, zeros, rot90
from numpy.linalg import norm

def add_marker(pose, name, color=[1,0,1], type=Marker.SPHERE, scale = [.02,.02,.02], points=None, frame = "world"):
        vis_pub = rospy.Publisher(name, Marker, queue_size=10)
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time() 
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = type
        marker.action = Marker.ADD
        
        if type == Marker.LINE_LIST:
            for point in points:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = point[2]
                marker.points.append(p)
        else:
            r = find_rotation_matrix_between_two_vectors([1,0,0], [0,0,1])
            rot = pose[0:3,0:3] * r
            pose2 = np.matrix(np.identity(4))
            pose2[0:3,0:3] = rot
            pose2[0:3,3] = pose[0:3,3]
            quat_pose = PoseConv.to_pos_quat(pose2)
            
            marker.pose.position.x = quat_pose[0][0]
            marker.pose.position.y = quat_pose[0][1]
            marker.pose.position.z = quat_pose[0][2]
            marker.pose.orientation.x = quat_pose[1][0]
            marker.pose.orientation.y = quat_pose[1][1] 
            marker.pose.orientation.z = quat_pose[1][2]
            marker.pose.orientation.w = quat_pose[1][3] 
            
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.a = .5
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        
        
        
        vis_pub.publish(marker)
        

def find_rotation_matrix_between_two_vectors(a,b):
        """!
            Returns a rotation matrix between vectors a and b
            @param a : A vector
            @param b : A vector
            @return R : A 3x3 rotation matrix
        """
        a = np.array(a).reshape(1,3)[0].tolist()
        b = np.array(b).reshape(1,3)[0].tolist()
        
        vector_orig = a / norm(a)
        vector_fin = b / norm(b)
                     
        # The rotation axis (normalised).
        axis = cross(vector_orig, vector_fin)
        axis_len = norm(axis)
        if axis_len != 0.0:
            axis = axis / axis_len
    
        # Alias the axis coordinates.
        x = axis[0]
        y = axis[1]
        z = axis[2]
    
        # The rotation angle.
        angle = acos(dot(vector_orig, vector_fin))
    
        # Trig functions (only need to do this maths once!).
        ca = cos(angle)
        sa = sin(angle)
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
    
def distance(a, b):
    return math.sqrt( sum([ (i-j)**2 for i,j in zip(a,b)]) )