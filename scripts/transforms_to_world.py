import rospy
import shelve
import numpy as np
import math

from scipy.optimize import minimize 
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from sensor_msgs.msg import JointState
from hrl_geom import pose_converter

psm1_robot = None
psm1_kin = None

psm2_robot = None
psm2_kin = None

ecm_robot = None
ecm_kin = None

E

def world_to_psm2():
    
    Twp2 = Twp1 * Tp12
    pass

def main():
    global psm1_kin,psm1_robot, psm2_kin, psm2_robot, ecm_kin, ecm_robot
    if psm1_robot is None:
        psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        psm1_kin = KDLKinematics(psm1_robot, psm1_robot.links[1].name, psm1_robot.links[-1].name)
    if psm2_robot is None:
        psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
        psm2_kin = KDLKinematics(psm2_robot, psm2_robot.links[1].name, psm2_robot.links[-1].name)
    if ecm_robot is None:
        ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        ecm_kin = KDLKinematics(ecm_robot, ecm_robot.links[3].name, ecm_robot.links[-1].name)
        
    
if __name__ == "__main__":
    main()