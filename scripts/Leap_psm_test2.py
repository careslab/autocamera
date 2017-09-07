#!/usr/bin/env python
# license removed for brevity
import rospy
import Leap, sys, thread, time
from sensor_msgs.msg._JointState import JointState

def talker():
    
    # Creating a controller and initializing parameters
    grabstrengthright = 0.0
    wristpitchright = 0.0
    wristrollright = 0.0
    wristyawright = 0.0
    outeryawright = 0.0
    outerinsertionright = 0.0
    outerpitchright = 0.0
    grabstrengthleft = 0.0
    wristpitchleft = 0.0
    wristrollleft = 0.0
    wristyawleft = 0.0
    outeryawleft = 0.0
    outerinsertionleft = 0.0
    outerpitchleft = 0.0
    controller = Leap.Controller()

    #Publishers    
    psm1_joint_angles = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=1)
    psm2_joint_angles = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(50) # 50hz

    #JointState Arrays
    msg1 = JointState()
    msg2 = JointState()

    #Do these continuously
    while not rospy.is_shutdown():

	# Get the most recent frame
        frame = controller.frame()
        # Get hands
        for hand in frame.hands:
        # Get the hand's normal vector and direction
	    normal = hand.palm_normal
	    direction = hand.direction
	    wrist = hand.wrist_position

	    #Right Hand
            if hand.is_right:
	        grabstrengthright = hand.grab_strength
	        wristpitchright = direction.pitch
                wristyawright = direction.yaw
                wristrollright = normal.roll
	        outeryawright = wrist[0]
	        outerinsertionright = wrist[1]
	        outerpitchright = wrist[2]
	    #Left Hand
	    if hand.is_left:	 
                grabstrengthleft = hand.grab_strength
	        wristpitchleft = direction.pitch
                wristyawleft = direction.yaw
                wristrollleft = normal.roll
		outeryawleft = wrist[0]
	        outerinsertionleft = wrist[1]
	        outerpitchleft = wrist[2]

	#Publish the JointStates
	msg1.position = [(outeryawright)/100.0, (30.0-outerpitchright)/100.0, 0.0, 0.0, 0.0, 0.0, 0.0, (250.0-outerinsertionright)/1000.0, wristrollright, wristpitchright, (wristyawright)*2.0, (1.0-grabstrengthright)*1.57 , 0.0, 0.0]
        msg1.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
        #rospy.loginfo(msg1)
        psm1_joint_angles.publish(msg1)
	
	msg2.position = [(outeryawleft)/100.0, (30.0-outerpitchleft)/100.0, 0.0, 0.0, 0.0, 0.0, 0.0, (250.0-outerinsertionleft)/1000.0, wristrollleft, wristpitchleft, (wristyawleft)*2.0, (1-grabstrengthleft)*1.57, 0.0, 0.0]
        msg2.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
        #rospy.loginfo(msg2)
        psm2_joint_angles.publish(msg2)
	
	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
