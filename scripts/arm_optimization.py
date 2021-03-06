import rospy
import shelve
import numpy as np
import math
import sys

from scipy.optimize import minimize 
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from sensor_msgs.msg import JointState
from hrl_geom import pose_converter

# MODE = 'ecm_psm1'  # 'ecm_psm1' or 'psm2_psm1'

MODE = None

psm1_robot = None
psm1_kin = None

psm2_robot = None
psm2_kin = None

ecm_robot = None
ecm_kin = None


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
    if name == "ecm":
        ecm_joint_angles = file['ecm_angles']
        data = []
        for angles in ecm_joint_angles:
            T_rene_condom = np.eye(4)
            T_rene_condom[2,3] = 0.0196  # Should probably be closer to [0.0192, 0.0193] m
            T = ecm_kin.forward( angles) * T_rene_condom    
            xyz = T[0:3,3]
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
    if objective_function.ecm_data == None:
        objective_function.ecm_data = compute_fk('ecm')
    
    T = pose_converter.PoseConv.to_homo_mat(xyzrpy)
    
    if objective_function.mode == 'ecm_psm1':
        pos_in_psm1rf = []
        for ecm_xyz in objective_function.ecm_data:
            ecm_xyz = np.insert(ecm_xyz,3,1)
            ecm_xyz = ecm_xyz.transpose()
            temp = T *  ecm_xyz
            pos_in_psm1rf.append(temp)
    else:
        pos_in_psm1rf = []
        for psm2_xyz in objective_function.psm2_data:
            psm2_xyz = np.insert(psm2_xyz,3,1)
            psm2_xyz = psm2_xyz.transpose()
            temp = T *  psm2_xyz
            pos_in_psm1rf.append(temp)
    
    diff = []
    for a,b in zip(objective_function.psm1_data, pos_in_psm1rf):
        a = [ i[0] for i in a.tolist()]
        b = [ i[0] for i in b.tolist()]
        diff.append(dist(a,b))
    return sum(diff)/len(diff)
    
# Static variables    
objective_function.psm1_data = None
objective_function.psm2_data = None
objective_function.ecm_data = None
objective_function.mode = MODE  # 'ecm_psm1' or 'psm2_psm1'

def find_everything_related_to_world(arm_name, xyzrpy):
    global psm1_kin,psm1_robot, psm2_kin, psm2_robot, ecm_kin, ecm_robot
    if len(xyzrpy) > 2:
        xyzrpy = [ tuple(xyzrpy[0:3]), tuple(xyzrpy[3:])]
    
    psm1_kin = KDLKinematics(psm1_robot, psm1_robot.links[0].name, psm1_robot.links[1].name) 
    Twp1 = psm1_kin.forward([])
            
    if arm_name == 'psm2':
        
        Tp12 = pose_converter.PoseConv.to_homo_mat(xyzrpy)
        
        Twp2 = Twp1 * Tp12
        Twp2_euler = pose_converter.PoseConv.to_pos_euler(Twp2)
        return Twp2_euler
    if arm_name == 'ecm':
        ecm_kin = KDLKinematics(ecm_robot, ecm_robot.links[1].name, ecm_robot.links[3].name)
        Tse = ecm_kin.forward([])
        Tp1E = pose_converter.PoseConv.to_homo_mat(xyzrpy)
        Tws = Twp1 * Tp1E * (Tse**-1)
        Tws_euler = pose_converter.PoseConv.to_pos_euler(Tws)
        
        return Tws_euler

def main():
    global psm1_kin,psm1_robot, psm2_kin, psm2_robot, ecm_kin, ecm_robot
    
    objective_function.mode = MODE
    
    if psm1_robot is None:
        psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        psm1_kin = KDLKinematics(psm1_robot, psm1_robot.links[1].name, psm1_robot.links[-1].name)
    if psm2_robot is None:
        psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
        psm2_kin = KDLKinematics(psm2_robot, psm2_robot.links[1].name, psm2_robot.links[-1].name)
    if ecm_robot is None:
        ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        ecm_kin = KDLKinematics(ecm_robot, ecm_robot.links[3].name, ecm_robot.links[-1].name)
        
    initial_guess = [ (.80,0.5,.3), (0.2,0.7,1.57)]
    res = minimize(objective_function, initial_guess, method='nelder-mead', options={'xtol':1e-12, 'disp':True, 'maxiter': 100000, 'maxfev':100000},)
    print(res)
    print(res.x)
    file.close()
    print(objective_function.mode)   
    if objective_function.mode == 'psm2_psm1':
        print('psm2 relative to world: ')
        v = find_everything_related_to_world('psm2', res.x)
 #       print("""xyz="{} {} {}" rpy="{} {} {}" """.format(v[0], v[1]) )
        print("""xyz="{0} {1} {2}" rpy="{3} {4} {5}" """.format(v[0][0],v[0][1],v[0][2],v[1][0],v[1][1],v[1][2]))
    if objective_function.mode == 'ecm_psm1':
        print('ecm relative to world: ')
        v = find_everything_related_to_world('ecm', res.x)
        print("""xyz="{0} {1} {2}" rpy="{3} {4} {5}" """.format(v[0][0],v[0][1],v[0][2],v[1][0],v[1][1],v[1][2]))
    
#     ecm_kin = KDLKinematics(ecm_robot, ecm_robot.links[0].name, ecm_robot.links[3].name)
#     psm1_kin = KDLKinematics(psm1_robot, psm1_robot.links[0].name, psm1_robot.links[1].name)
#     
#     print pose_converter.PoseConv.to_homo_mat( [ tuple(res.x[0:3]), list(res.x[3:])])
#     
#     print ( (psm1_kin.forward([])**-1) * ecm_kin.forward([]) )

    
if __name__ == "__main__":
    argv = sys.argv
    if len(argv) > 1:
        if 'ecm' in argv[1].lower():
            MODE = 'ecm_psm1'
        elif 'psm' in argv[1].lower():
            MODE = 'psm2_psm1'
    if MODE is None:
        print("""
            Usage: python arm_optimization [ecm | psm]
            """)
        exit(0)
        
    main()
