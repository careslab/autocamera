#************** This is the vive subscriber for the left hand position 

# Ros libraries and defs
import roslib;
import rospy
from sensor_msgs.msg._JointState import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
msg1 = JointState()
psm2_joint_angles = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=1) #left psm
vis_pub = rospy.Publisher("/key_holeLM", Marker, queue_size=1)
	
# Kinematics Libraries #
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics

# Python Maths things
import numpy as np
from math import pi
import PSM_orientation_fncs2 as pof

# Generate the robot variables
psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')


# Make some joints
global q_out
global qList
qList=[0.0, 0., .1,0]      # Initial insertion joint is set to 0.1 so that the arm can have sufficient space

# Joint Limits
q_min=[-1.5707,-0.7854,0,-2.2689]
q_max=[1.5707,0.7854,0.24,2.2689]


# Default deltas
dx= 0.0
dy= 0.0
dz= 0.0

# Kinematics Solvers
psm2_kin = KDLKinematics(psm2_robot,psm2_robot.links[0].name,psm2_robot.links[11].name)
ecm_kin  = KDLKinematics(ecm_robot,ecm_robot.links[0].name,ecm_robot.links[-1].name)

initialFrame=[]
desiredFrame=np.eye(4)

print "initial joints\n",qList

global frameECM
frameECM=ecm_kin.forward([0.0, 0, 0, 0])
frameECMinv=np.linalg.inv(frameECM)

# Calculate the keyhole and mark it in 3d space
key_hole = pof.find_keyholeR()
print "Key hole is at ", key_hole
print " in the world frame\n"

marker = Marker()
marker.header.frame_id = "world"
marker.header.stamp = rospy.Time() 
marker.ns = "my_namespace"
marker.id = 0
marker.type = 2
marker.action = 0

marker.pose.position.x = key_hole[0]
marker.pose.position.y = key_hole[1]
marker.pose.position.z = key_hole[2]

marker.scale.x = 0.015
marker.scale.y = 0.015
marker.scale.z = 0.015

marker.color.a = .75
marker.color.r = 0
marker.color.g = 0.2
marker.color.b = 0.2

def cb(d):	
	
	global qList
	#print '************************* \n   Function Called\n*************************'
	initialFrame=psm2_kin.forward(qList)  # Do the forward Kinematics for the initial angles
	
#      The desired frame is the initial frame plus the delta
	dx=(d.linear.x)/10;
	dy=(d.linear.y)/10;
	dz=(d.linear.z)/10;
	
	current=np.array([initialFrame[0,3],initialFrame[1,3],initialFrame[2,3],1])
	ecmFramePos=np.matmul(frameECMinv,current)
	ecmFramePos[0,0] = ecmFramePos[0,0]+ dx
	ecmFramePos[0,1] = ecmFramePos[0,1]+ dy
	ecmFramePos[0,2] = ecmFramePos[0,2]+ dz
	worldFramePos=np.matmul(frameECM,np.transpose(ecmFramePos)) 
	
	desiredFrame[0,3] = worldFramePos[0,0]
	desiredFrame[1,3] = worldFramePos[1,0]
	desiredFrame[2,3] = worldFramePos[2,0]
	# Take the inverse again to put us back in the robot base frame. This hasn't changed the rotation yet.


	#These lines create the keyhole/inital_position and keyhole/desired_position vectors
	initialP=np.array([initialFrame[0,3] ,initialFrame[1,3] ,initialFrame[2,3]]);   
	desiredP=np.array([desiredFrame[0,3] ,desiredFrame[1,3] ,desiredFrame[2,3]]);
	a= key_hole - initialP
	b= key_hole - desiredP

	dif=((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)**0.5 # Calculate the difference between the initial and desired vectors
	if dif < 0.000000001:			# If the difference is sufficiently small, just assume its the same
		rot=[[1,0,0],[0,1,0],[0,0,1]]
	else:	
		rot=pof.find_rotation(a,b)	# Otherwise calculate the rotation between the vectors

	finalRot=pof.apply_rotation(initialFrame, rot) 	# Multiply the initial frame orientation by the rotation calculated above
	
	# Assign the final rotation to the desired frame
	desiredFrame[0,0]=finalRot[0,0]
	desiredFrame[0,1]=finalRot[0,1]
	desiredFrame[0,2]=finalRot[0,2]
	desiredFrame[1,0]=finalRot[1,0]
	desiredFrame[1,1]=finalRot[1,1]
	desiredFrame[1,2]=finalRot[1,2]
	desiredFrame[2,0]=finalRot[2,0]
	desiredFrame[2,1]=finalRot[2,1]
	desiredFrame[2,2]=finalRot[2,2]
	
	
	# Solve the Inverse Kinematics with the joint limits
	q_out = psm2_kin.inverse(desiredFrame,qList,q_min,q_max)
	print "q out (None is bad)\n",q_out

	# Publish the angles q_out
	
	msg1.position = [q_out[0],q_out[1], 0.0, 0.0, 0.0, 0.0, 0.0,q_out[2],d.angular.x,d.angular.y,d.angular.z]
	msg1.name = ['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw']
	rospy.loginfo(msg1)
	psm2_joint_angles.publish(msg1)	
	vis_pub.publish(marker)	# Publish keyhole position. This doesnt need to be done every loop, but it does need to be placed in a fnc
	
	qList=q_out	# Calculated joints become the next initial joints


def listener():
	rospy.init_node('leftJointPub', anonymous=True)
#	rospy.Subscriber("deltas", Twist, cb)	
	rospy.Subscriber("/vive/left_hand", Twist, cb,queue_size=1) 	# queue size must be 1 or it lags and
								  	# tries to publish every message it gets
	rate = rospy.Rate(60) # 60hz
	print('Listener running')
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
