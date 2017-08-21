#!/usr/bin/env python
import rospy
from sensor_msgs.msg._JointState import JointState

def callback(position):
    rospy.loginfo("Joint Positions are %s", position.position)
    rospy.loginfo("Joint Names are %s", position.name)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/dvrk_psm1/joint_states', JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

