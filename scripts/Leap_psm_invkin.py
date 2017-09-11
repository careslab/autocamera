#!/usr/bin/env python
# license removed for brevity
import rospy
import Leap, sys, thread, time
from sensor_msgs.msg._JointState import JointState
import numpy as np
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics

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

    psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
    psm1_kin = KDLKinematics(psm1_robot, psm1_robot.links[0].name, psm1_robot.links[-1].name)
    psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
    psm2_kin = KDLKinematics(psm2_robot, psm2_robot.links[0].name, psm2_robot.links[-1].name)

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
		grabstrengthright = (1.0-grabstrengthright)*1.57
		if grabstrengthright < 0.0:
		    grabstrengthright = 0.0
		elif grabstrengthright > 1.57:
		    grabstrengthright = 1.57

	        wristpitchright = direction.pitch
		if wristpitchright < -1.57:
		    wristpitchright = -1.57
		elif wristpitchright > 1.57:
		    wristpitchright = 1.57

                wristyawright = direction.yaw
		wristyawright = (wristyawright)*2.0
		if wristyawright < -1.4:
		    wristyawright = -1.4
		elif wristyawright > 1.4:
		    wristyawright = 1.4

                wristrollright = normal.roll
		if wristrollright < -2.27:
		    wristrollright = -2.27
		elif wristrollright > 2.27:
		    wristrollright = 2.27

	        outeryawright = wrist[0]
		outeryawright = (1.5-outeryawright)/100.0
		if outeryawright < -1.57:
		    outeryawright = -1.57
		elif outeryawright > 1.57:
		    outeryawright = 1.57

	        outerinsertionright = wrist[2]
		outerinsertionright = ((-0.0218182*outerinsertionright)+0.218182)
		if outerinsertionright < 0.0:
		    outerinsertionright = 0.0
		elif outerinsertionright > 0.24:
		    outerinsertionright = 0.24

	        outerpitchright = wrist[1]
		outerpitchright = 0.79-((0.0079*outerpitchright)-1.185)
		if outerpitchright < -0.79:
		    outerpitchright = -0.79
		elif outerpitchright > 0.79:
		    outerpitchright = 0.79

	    #Left Hand
	    if hand.is_left:	 

                grabstrengthleft = hand.grab_strength
		grabstrengthleft = (1.0-grabstrengthleft)*1.57
		if grabstrengthleft < 0.0:
		    grabstrengthleft = 0.0
		elif grabstrengthleft > 1.57:
		    grabstrengthleft = 1.57

	        wristpitchleft = direction.pitch
		if wristpitchleft < -1.57:
		    wristpitchleft = -1.57
		elif wristpitchleft > 1.57:
		    wristpitchleft = 1.57

                wristyawleft = direction.yaw
		wristyawleft = (wristyawleft)*2.0
		if wristyawleft < -1.4:
		    wristyawleft = -1.4
		elif wristyawleft > 1.4:
		    wristyawleft = 1.4

                wristrollleft = normal.roll
		if wristrollleft < -2.27:
		    wristrollleft = -2.27
		elif wristrollleft > 2.27:
		    wristrollleft = 2.27

		outeryawleft = wrist[0]
		outeryawleft = (1.5-outeryawleft)/100.0
		if outeryawleft < -1.57:
		    outeryawleft = -1.57
		elif outeryawleft > 1.57:
		    outeryawleft = 1.57

	        outerinsertionleft = wrist[2]
		outerinsertionleft = ((-0.0218182*outerinsertionleft)+0.218182)
		if outerinsertionleft < 0.0:
		    outerinsertionleft = 0.0
		elif outerinsertionleft > 0.24:
		    outerinsertionleft = 0.24

	        outerpitchleft = wrist[1]
		outerpitchleft = 0.79-((0.0079*outerpitchleft)-1.185)
		if outerpitchleft < -0.79:
		    outerpitchleft = -0.79
		elif outerpitchleft > 0.79:
		    outerpitchleft = 0.79


	#Publish the JointStates
	msg1.position = [outeryawright, outerpitchright, 0.0, 0.0, 0.0, 0.0, 0.0, outerinsertionright, wristrollright, wristpitchright, wristyawright, grabstrengthright, 0.0, 0.0]
        msg1.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
        #rospy.loginfo(msg1)
        psm1_joint_angles.publish(msg1)
	
	msg2.position = [outeryawleft, outerpitchleft, 0.0, 0.0, 0.0, 0.0, 0.0, outerinsertionleft, wristrollleft, wristpitchleft, wristyawleft, grabstrengthleft, 0.0, 0.0]
        msg2.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
        #rospy.loginfo(msg2)
        psm2_joint_angles.publish(msg2)
	
	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
