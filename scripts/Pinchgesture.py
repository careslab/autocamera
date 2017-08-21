import rospy
import Leap, sys, thread, time
from Leap import CircleGesture,  KeyTapGesture, ScreenTapGesture, SwipeGesture
from std_msgs.msg import String

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
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown(): 
        motion = "Pinch Strength is is %s" % mylistener
        rospy.loginfo(motion)
        pub.publish(motion)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
