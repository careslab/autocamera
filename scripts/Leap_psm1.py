#!/usr/bin/env python
# license removed for brevity
import rospy
import Leap, sys, thread, time
from Leap import CircleGesture
from sensor_msgs.msg._JointState import JointState
import itertools

class MyListener(Leap.Listener):

    def on_init(self, controller):
	print "Initialized"
	self.clockwiseness = ""

    def on_connect(self, controller):
        print "Connected"
	# Enable gesture
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE)

    def on_frame(self, controller):
        # Get the most recent frame
        frame = controller.frame()
	# Get gesture
        for gesture in frame.gestures():
            if gesture.type == Leap.Gesture.TYPE_CIRCLE:
                circle = CircleGesture(gesture)
		# Determine clock direction using the angle between the pointable and the circle normal
                if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/2:
                    self.clockwiseness = "clockwise"
                else:
                    self.clockwiseness = "counterclockwise"

    def __str__(self):
        return self.clockwiseness

def frange(x, y, jump):
  while x < y:
    yield x
    x += jump

def frangerev(a, b, jump):
  while a > b:
    yield a
    a -= jump

def talker():
    # Creating a listener and controller
    mylistener = MyListener()
    controller = Leap.Controller()
    # Have the listener receive events from the controller
    controller.add_listener(mylistener)
    psm1_joint_angles = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	if str(mylistener) == "clockwise":
	    for i, j in itertools.product(frange(0.0,1.5707,0.01), frange(0.0,0.78535,0.01)):
		#Jaw Open
		joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i, j, -j]
		msg = JointState()
        	msg.position = joint_angles
        	msg.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
        	rospy.loginfo(msg)
        	psm1_joint_angles.publish(msg)
	if str(mylistener) == "counterclockwise":
	    for i, j in itertools.product(frangerev(1.5707,0.0,0.01), frangerev(0.78535,0.0,0.01)):
		#Jaw Open
		joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i, j, -j]
		msg = JointState()
        	msg.position = joint_angles
        	msg.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
        	rospy.loginfo(msg)
        	psm1_joint_angles.publish(msg)
	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
