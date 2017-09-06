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
	self.grabstrengthleft = 0.0

	self.wristpitchright = 0.0
	self.wristrollright = 0.0
	self.wristyawright = 0.0
	self.outeryaw = 0.0
	self.outerinsertion = 0.0
	self.outerpitch = 0.0

	self.wristpitchleft = 0.0
	self.wristrollleft = 0.0
	self.wristyawleft = 0.0

    def on_connect(self, controller):
        print "Connected"

    def on_frame(self, controller):
        # Get the most recent frame
        frame = controller.frame()
	# Get hands
	for hand in frame.hands:
	# Get the hand's normal vector and direction
	    normal = hand.palm_normal
	    direction = hand.direction
	    wrist = hand.wrist_position

            if hand.is_right:
		self.grabstrengthright = 0.0#hand.grab_strength
		self.wristpitchright = 0.0#direction.pitch
                self.wristyawright = 0.0#direction.yaw
                self.wristrollright = 0.0#normal.roll
		self.outeryaw = wrist[0]
		self.outerinsertion = wrist[1]
		self.outerpitch = 0.0#wrist[2]

	    #if hand.is_left:	 
             #   self.grabstrengthleft = hand.grab_strength
	      #  self.wristpitchleft = direction.pitch
		#self.wristpitchleft = 0.0
                #self.wristyawleft = direction.yaw
		#self.wristyawleft = 0.0
                #self.wristrollleft = normal.roll
		#print 'Wristrollleft %f' %self.wristrollleft
    	
    def _repr_(self):
        return [self.grabstrengthright, self.wristpitchright, self.wristrollright, self.wristyawright, self.outeryaw, self.outerinsertion, self.outerpitch, self.wristpitchleft, self.wristrollleft, self.wristyawleft]

def talker():
    # Creating a listener and controller
    mylistener = MyListener()
    controller = Leap.Controller()
    
    # Have the listener receive events from the controller
    controller.add_listener(mylistener)
    
    psm1_joint_angles = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=1)
    psm2_joint_angles = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(50) # 50hz

    msg1 = JointState()
    msg2 = JointState()

    while not rospy.is_shutdown():

	msg1.position = [(mylistener.outeryaw)/100, (30-mylistener.outerpitch)/100, 0.0, 0.0, 0.0, 0.0, 0.0, (250-mylistener.outerinsertion)/1000, mylistener.wristrollright, mylistener.wristpitchright, (mylistener.wristyawright)*2.0, (1.0-mylistener.grabstrengthright)*1.57 , 0.0, 0.0]
        msg1.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
        rospy.loginfo(msg1)
        psm1_joint_angles.publish(msg1)
	
	#msg2.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, mylistener.wristrollleft, mylistener.wristpitchleft, (mylistener.wristyawleft)*2, (1-mylistener.grabstrengthleft)*1.57, 0.0, 0.0]
        #msg2.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
        #rospy.loginfo(msg2)
        #psm2_joint_angles.publish(msg2)
	
	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
