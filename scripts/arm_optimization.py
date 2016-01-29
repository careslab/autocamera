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

file = shelve.open('joint_angles.db')

# Compute the forward kinematics, and retrieve the x,y,z for each joint angle that we read from file
def compute_fk(name):
    global file
    if name == "psm1":
        psm1_joint_angles = file['psm1_angles']
        data = []
        for angles in psm1_joint_angles:
            xyz,_ = psm1_kin.FK(angles[:-1])
            data.append(xyz)
        return data
    if name == "psm2":
        psm2_joint_angles = file['psm2_angles']
        data = []
        for angles in psm2_joint_angles:
            xyz,_ = psm2_kin.FK(angles[:-1])
            data.append(xyz)
        return data

def dist(a,b):
    return math.sqrt( sum([ (i-j)**2 for i,j in zip(a,b)]))

# This function is supposed to be fed to scipy 
def objective_function(xyzrpy):
    if len(xyzrpy) > 2:
        xyzrpy = [ tuple(xyzrpy[0:3]), tuple(xyzrpy[3:])]
    if objective_function.psm1_data == None:
        objective_function.psm1_data = compute_fk('psm1')
    if objective_function.psm2_data == None:
        objective_function.psm2_data = compute_fk('psm2')
    
    T = pose_converter.PoseConv.to_homo_mat(xyzrpy)
    psm2pos_in_psm1rf = []
    for psm2_xyz in objective_function.psm2_data:
        psm2_xyz = np.insert(psm2_xyz,3,1)
        psm2_xyz = psm2_xyz.transpose()
        temp = T *  psm2_xyz
        psm2pos_in_psm1rf.append(temp)
    
    diff = []
    for a,b in zip(objective_function.psm1_data, psm2pos_in_psm1rf):
        a = [ i[0] for i in a.tolist()]
        b = [ i[0] for i in b.tolist()]
        diff.append(dist(a,b))
    return sum(diff)/len(diff)
    
# Static variables    
objective_function.psm1_data = None
objective_function.psm2_data = None


def main():
    global psm1_kin,psm1_robot, psm2_kin, psm2_robot
    if psm1_robot is None:
        psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        psm1_kin = KDLKinematics(psm1_robot, psm1_robot.links[1].name, psm1_robot.links[-1].name)
    if psm2_robot is None:
        psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
        psm2_kin = KDLKinematics(psm2_robot, psm2_robot.links[1].name, psm2_robot.links[-1].name)
    
    initial_guess = [ (.80,0.5,.3), (0.2,0.7,1.57)]
    res = minimize(objective_function, initial_guess, method='nelder-mead', options={'xtol':1e-10, 'disp':True, 'maxiter': 100000, 'maxfev':100000},)
    print(res)
    print(res.x)
    file.close()
    
if __name__ == "__main__":
    main()