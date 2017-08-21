#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg._JointState import JointState

def frange(x, y, jump):
    while x < y:
	yield x
	x += jump

def talker():
    ecm_sim = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    	for x in frange(-1.2,0.7,0.001):
		joint_angles = [x, -0.00017453292342706206, -0.00017453292342706206, 0.00017453292342706206, -0.00017453292342706206, 0.0, 0.0]
		print x
		msg = JointState()
        	msg.position = joint_angles
        	msg.name = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
        	rospy.loginfo(msg)
        	ecm_sim.publish(msg)
	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


