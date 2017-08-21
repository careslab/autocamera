#!/usr/bin/env python
# license removed for brevity
import rospy
import Leap, sys, thread, time
from Leap import CircleGesture
from sensor_msgs.msg._JointState import JointState

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
		print clockwiseness

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
    ecm_sim = rospy.Publisher('/dvrk_ecm/joint_states_robot', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	if str(mylistener) == "clockwise":
	    for x in frange(-1.2,0.7,0.001):
	    	joint_angles = [x, -0.00017453292342706206, -0.00017453292342706206, 0.00017453292342706206, -0.00017453292342706206, 0.0, 0.0]
	        msg = JointState()
                msg.position = joint_angles
                msg.name = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
	        rospy.loginfo(msg)
                ecm_sim.publish(msg)
	if str(mylistener) == "counterclockwise":
	    for z in frangerev(0.7,-1.2,0.001):
		joint_angles = [z, -0.00017453292342706206, -0.00017453292342706206, 0.00017453292342706206, -0.00017453292342706206, 0.0, 0.0]
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
