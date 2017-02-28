import rosbag
import rospy
import sys
import threading
import os
import time

from cisst_msgs.msg import vctDoubleVec

from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState
from rospy.timer import sleep

# ARM_NAMES = ['MTML']#, 'MTMR', 'PSM1', 'PSM2', 'ECM']
# ARM_NAME = 'MTMR'

class bag_writer:
    def __init__(self, arm_names):
        if type(arm_names) == type(""):
            arm_names = [arm_names]
        # initialize a bag 
        self.bag = rosbag.Bag('test.bag', 'w')
        self.arm_names = arm_names
        # The topics we want to record
#         self.topics = '/dvrk/{}/state_joint_current'.format(ARM_NAME)
        self.topics = { arm_name:'/dvrk/{}/state_joint_current'.format(arm_name) for arm_name in self.arm_names}
        self.out_topics_hw = {arm_name : '/dvrk/{}/set_position_joint'.format(arm_name) for arm_name in self.arm_names}
        self.out_topics_sim = {arm_name : '/dvrk_{}/joint_states_robot'.format(arm_name.lower()) for arm_name in self.arm_names}
        
#         self.out_topics = self.out_topics_sim
        self.out_topics = self.topics
        print self.topics
        # We have to initialize a ros node if we want to subsribe or publish messages
        rospy.init_node('rosbag_test_node')
        
        for arm_name in self.arm_names:
            eval("rospy.Subscriber('{}', JointState, self.cb_{})".format(self.topics[arm_name], arm_name))
            sleep(.1)
    
        rospy.spin()
        
    def cb_MTML(self, msg):
        self.cb(self.out_topics['MTML'], msg)
    def cb_MTMR(self, msg):
        self.cb(self.out_topics['MTMR'], msg)
    def cb_PSM1(self, msg):
        self.cb(self.out_topics['PSM1'], msg)
    def cb_PSM2(self, msg):
        self.cb(self.out_topics['PSM2'], msg)
    def cb_ECM(self, msg):
        self.cb(self.out_topics['ECM'], msg)
                    
    def cb(self,topic, msg):
        try:
            self.bag.write(topic, msg) # record the msg
        except Exception:
            pass
    
    # This is the destructor for this python class    
    def __del__(self):
        self.bag.close()
    
class bag_reader:
    
    def __init__(self, my_arm_name):
        self.arm_name = my_arm_name
        rospy.init_node('rosbag_test_node')
        self.bag = rosbag.Bag('test.bag')
        # The topics we used for recording
        self.topics = { arm_name:'/dvrk/{}/state_joint_current'.format(self.arm_name)}# for arm_name in self.arm_name}
#         self.topic = '/dvrk/{}/state_joint_current'.format(ARM_NAME)
        
        # set robot state before we can publish
#         self.robot_state = rospy.Publisher('/dvrk/{}/set_robot_state'.format(ARM_NAME), String, latch=True, queue_size=1)
        
        # The topics we want to publish to
        self.pub_hw = {self.topics[arm_name] : rospy.Publisher('/dvrk/{}/set_position_joint'.format(self.arm_name), JointState, latch=True, queue_size=1)}# for arm_name in self.arm_name}
        self.pub_sim = {self.topics[arm_name] : rospy.Publisher('/dvrk_{}/joint_states_robot'.format(self.arm_name.lower()), JointState, queue_size=10 )}# for arm_name in self.arm_name}
        
        self.read_count = 0

        
        
    def set_state(self, topic, state_name):
        arm_name = None
        print self.topics
        for a, t in self.topics.items():
            if t==topic:
                arm_name = a
                break
        print 'arm_name is ' + arm_name
        p = rospy.Publisher('/dvrk/{}/set_robot_state'.format(arm_name), String, latch=True, queue_size=1)
        p.publish(state_name)
            
    
    def read(self, device):
        """
            device : "simulation" or "hardware"
        """
        self.read_topic( self.topics[self.arm_name], device)
            
    def read_topic(self, my_topic, device):
        device = device.lower()
        
        print 'my_topic = ' + my_topic.__str__() 
        
        
        time_diff = rospy.Duration(1)
        time_previous_message = None
        
        # FIXME:  The current timing code will probably have an increasing delay as messages are played.
        #         We should fix this by basing the sleep duration on the difference between
        #         the current wall clock and the desired time of the message
        
        print( '\nmsg count = ' + self.bag.get_message_count().__str__())

        bag_messages = self.bag.read_messages( topics=[my_topic])
        
        for topic, msg, time_current_message in bag_messages:
            if device == "hardware":
                if self.read_count == 0:
                    self.set_state(topic, 'DVRK_POSITION_GOAL_JOINT')
                elif self.read_count == 3:
                    self.set_state(topic, 'DVRK_POSITION_JOINT')
                
            self.read_count = self.read_count + 1
#             print(self.read_count)
            
            # the timestamp of each recorded message is saved as time_current_message
            if time_previous_message != None:
                time_diff = time_current_message-time_previous_message
            
            temp_t = time.time()
            
            sleep(time_diff)
#             s = raw_input('\nPress any key to continue...(except q)')
#             if s == 'q':
#                 return
            print('time.time says {}\n'.format(time.time() - temp_t))
            print( 'time_diff = {} '.format(time_diff.to_sec() ) )
            
#             print('time_previous_message = {}, rostime = {}'.format(time_previous_message, rospy.rostime.get_rostime()))
            time_previous_message= time_current_message
            if 'PSM1' in topic or 'PSM2' in topic:
                msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
            elif 'ECM' in topic:
                msg.name = ['outer_yaw', 'outer_pitch', 'insertion', 'outer_roll']
            elif 'MTM' in topic:
                msg.name = ['outer_yaw', 'shoulder_pitch', 'shoulder_pitch_parallel', 'elbow_pitch', 'wrist_platform', 'wrist_pitch', 'wrist_yaw', 'wrist_roll']
            # publish
            if self.read_count < 3 and device == "hardware": 
                print('HOMING..\n')
                if self.arm_name in topic:
                    print(self.arm_name + topic)
                    rospy.Publisher('/dvrk/{}/set_position_goal_joint'.format(self.arm_name), JointState, queue_size=10).publish(msg)
                sleep(2)
            else:
                if device == "simulation":
                    self.pub_sim[topic].publish(msg)
                if device == "hardware":
                    msg.position = [ round(i,4) for i in msg.position]
                    self.pub_hw[topic].publish(msg)
            
    def __del__(self):
        self.bag.close()
        

def wait_for_exit():
    s = raw_input('\nPress any key to exit...')  
#     os._exit
    print('Press ctrl+c to exit..\n')
    return      
    exit(0)
    
VALID_ARM_NAMES = ['MTML', 'MTMR', 'PSM1', 'PSM2', 'ECM']

# The mode can be 'playback' for read, and 'record' for write
mode = 'record' 
if __name__=='__main__':
    usage = \
    """
Usage:
    python rosbag_example.py record [ MTML | MTMR | PSM1 | PSM2 | ECM ]
        
    python rosbag_example.py playback [ hardware | simulation ] [ MTML | MTMR | PSM1 | PSM2 | ECM ] 
    
    python rosbag_example.py -h : show this prompt
    
Example:
    python rosbag_example.py record MTML
    python rosbag_example.py playback hardware MTML
    """
    if len(sys.argv) < 2 or sys.argv[1] == '-h':
        print(usage)
        exit(0)
    mode = sys.argv[1].lower()
    print sys.argv.__str__()
    if not(mode == 'record' or mode == 'playback'):
        print('invalid argument\n')
        exit(0)
    if len(sys.argv) < 3:
        print('Missing arm name(s). Choose one (or more) of the following:\n {}, {}, {}, {}, {}'.format(*VALID_ARM_NAMES))
        exit(-1)
    t=threading.Thread(target=wait_for_exit)
    t.start()
    
    if mode=='record':    
        for i in range(2,len(sys.argv)-1):
            if sys.argv[i] not in VALID_ARM_NAMES:
                print('Invalid arm name : ' + sys.argv[i] + '\n')
                exit(-1)
                        
        print('Recording...')
        arm_names = sys.argv[2:]
        bw = bag_writer(arm_names)
    elif mode == 'playback':
        device = sys.argv[2]
        arm_name = sys.argv[3]
        if len(sys.argv) >4:
            print('Cannot play more than 1 arm at the moment')
            exit(-1)
        if device.lower() not in ["simulation", "hardware"]:
            print('Incorrect playback device name. Choose "hardware" or "simulation"\n')
            exit(-1)
        print('Playing...')
        
        br = bag_reader(arm_name)
        br.read(device)
        
    if not t.isAlive():
        print('exiting\n')
        exit(0)