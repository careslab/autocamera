import rospy
import Leap, sys, thread, time
from Leap import CircleGesture
from std_msgs.msg import String

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
		#print clockwiseness

    def __str__(self):
        return self.clockwiseness

def talker():
    # Creating a listener and controller
    mylistener = MyListener()
    controller = Leap.Controller()
    # Have the listener receive events from the controller
    controller.add_listener(mylistener)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown(): 
        motion = "Direction of circle is %s" % mylistener
        rospy.loginfo(motion)
        pub.publish(motion)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
