#!/usr/bin/env python
# license removed for brevity
import rospy
import Leap, sys, thread, time
from Leap import CircleGesture
from sensor_msgs.msg._JointState import JointState

class MyListener(Leap.Listener):

    def on_init(self, controller):
	print "Initialized"
	self.grabstrengthright = 0.0
	self.pinchstrengthleft = 0.0
	self.wristpitch = 0.0
	self.wristroll = 0.0
	self.wristyaw = 0.0

    def on_connect(self, controller):
        print "Connected"

    lastProcessedFrameID = 0
    def nextFrame(self, controller):
        currentID = controller.frame().id
    	for history in range(0, currentID - self.lastProcessedFrameID):
            self.processFrame(controller.frame(history))
            self.lastProcessedFrameID = currentID
    def processFrame(self, frame):
        if(frame.is_valid):

    #def on_frame(self, controller):
    #if(controller.is_connected):
        # Get the most recent frame
            frame = controller.frame()
	# Get hands
	    for hand in frame.hands:
                if hand.is_right:
		    self.grabstrengthright = hand.grab_strength
	    #if hand.is_left:
		#self.pinchstrengthleft = hand.pinch_strength self.pinchstrengthleft,
	 # Get the hand's normal vector and direction
                    normal = hand.palm_normal
		    direction = hand.direction
	            self.wristpitch = direction.pitch
                    self.wristyaw = direction.yaw
                    self.wristroll = normal.roll
    	
    def _repr_(self):
        return [self.grabstrengthright, self.wristpitch, self.wristroll, self.wristyaw]

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
	i = mylistener
	msg1 = JointState()
	msg1.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i.wristroll, i.wristpitch, (i.wristyaw)*2, (1-i.grabstrengthright)*1.57 , 0.0, 0.0]
        msg1.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
        rospy.loginfo(msg1)
        psm1_joint_angles.publish(msg1)
	#msg2 = JointState()  (1-i.grabstrengthright)*1.57  (i.wristyaw)*2
	#msg2.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (1-i.pinchstrengthleft)*1.57, 0.0, 0.0]
        #msg2.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
        #rospy.loginfo(msg2)
        #psm2_joint_angles.publish(msg2)
	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
