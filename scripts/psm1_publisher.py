#!/usr/bin/env python
# license removed for brevity
import itertools
import rospy
from sensor_msgs.msg._JointState import JointState

def frange(x, y, jump):
    while x < y:
	yield x
	x += jump

def talker():
    psm1_joint_angles = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	for i, j in itertools.product(frange(0.0,1.5707,0.01), frange(0.0,0.78535,0.01)):
		#Jaw Open
		joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i, j, -j]
		msg = JointState()
        	msg.position = joint_angles
        	msg.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
        	rospy.loginfo(msg)
        	psm1_joint_angles.publish(msg)

		#Jaw Close
		#joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		#msg = JointState()
        	#msg.position = joint_angles
        	#msg.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
        	#rospy.loginfo(msg)
        	#psm1_joint_angles.publish(msg)
	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

