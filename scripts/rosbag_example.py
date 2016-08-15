import rosbag
import rospy

from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState
from rospy.timer import sleep

class bag_writer:
    
    def __init__(self):
        # initialize a bag 
        self.bag = rosbag.Bag('test.bag', 'w')
        
        # The topic we want to record
        self.topic = '/dvrk_psm1/joint_states'
        
        # We have to initialize a ros node if we want to subsribe or publish messages
        rospy.init_node('rosbag_test_node')
        
        rospy.Subscriber(self.topic, JointState, self.cb)
        
        rospy.spin()
        
    def cb(self,msg):
        try:
            self.bag.write(self.topic, msg) # record the msg
        except Exception:
            pass
    
    # This is the destructor for this python class    
    def __del__(self):
        self.bag.close()
    
class bag_reader:
    
    def __init__(self):
        rospy.init_node('rosbag_test_node')
        self.bag = rosbag.Bag('test.bag')
        # The topic we used for recording
        self.topic = '/dvrk_psm1/joint_states'
        
        # The topic we want to publish to
        self.pub = rospy.Publisher('/dvrk_psm1/joint_states_robot', JointState, queue_size=10)
        
    def read(self):
        time_diff = 1
        t1 = None
        for topic, msg, t in self.bag.read_messages( topics=[self.topic]):
            # the timestamp of each recorded message is saved as t
            if t1 != None:
                time_diff = t-t1
            
            sleep(time_diff)
            t1 = t
            # publish to the simulation
            self.pub.publish(msg)
            
    def __del__(self):
        self.bag.close()
        
        

# The mode can be 'r' for read, and 'w' for write
mode = 'r' 
if __name__=='__main__':
    if mode=='w':
        bw = bag_writer()
    elif mode == 'r':
        br = bag_reader()
        br.read()
