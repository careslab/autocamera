from __common_imports__ import *

#rospy.init_node('whatever')
psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
psm2_kin = KDLKinematics(psm2_robot, psm2_robot.links[0].name, psm2_robot.links[-1].name)


