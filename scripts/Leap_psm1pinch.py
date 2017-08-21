#!/usr/bin/env python
# license removed for brevity
import rospy
import Leap, sys, thread, time
from Leap import CircleGesture
from sensor_msgs.msg._JointState import JointState

class MyListener(Leap.Listener):

    def on_init(self, controller):
	print "Initialized"
	self.pinchstrength = 0.0

    def on_connect(self, controller):
        print "Connected"

    def on_frame(self, controller):
        # Get the most recent frame
        frame = controller.frame()
	# Get hands
        for hand in frame.hands:
	    self.pinchstrength = hand.pinch_strength

    def __str__(self):
	return str(float(self.pinchstrength))


def talker():
    # Creating a listener and controller
    mylistener = MyListener()
    controller = Leap.Controller()
    # Have the listener receive events from the controller
    controller.add_listener(mylistener)
    psm1_joint_angles = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=10)
    #psm2_joint_angles = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	i = float(str(mylistener))
	msg = JointState()
	msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (1-i)*1.57, 0.0, 0.0]
        msg.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
        rospy.loginfo(msg)
        psm1_joint_angles.publish(msg)
        #psm2_joint_angles.publish(msg)
	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
