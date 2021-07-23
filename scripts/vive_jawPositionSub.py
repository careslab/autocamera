#************** This is the subscriber for the jaw state for the right and left jaws for the vive system 

# Ros libraries and defs
import roslib;
import rospy

from sensor_msgs.msg._JointState import JointState
from std_msgs.msg import Float32

psm1_jaw = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=1) #Right psm
psm2_jaw = rospy.Publisher('/dvrk_psm2/joint_states_robot', JointState, queue_size=1) #Left psm
msg1 = JointState()
msg2 = JointState()



def cbR(d1):	
	msg1.position = [d1.data]
	msg1.name = ['jaw']
	rospy.loginfo(msg1)
	psm1_jaw.publish(msg1)	

def cbL(d2):	
	msg2.position = [d2.data]
	msg2.name = ['jaw']
	rospy.loginfo(msg2)
	psm2_jaw.publish(msg2)		

def listener():
	rospy.init_node('JawPublisher', anonymous=True)
	rospy.Subscriber("/vive/right_jaw", Float32, cbR,queue_size=1)
	rospy.Subscriber("/vive/left_jaw", Float32, cbL,queue_size=1)
	rate = rospy.Rate(60) # 60hz
	print('Listener running')
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
