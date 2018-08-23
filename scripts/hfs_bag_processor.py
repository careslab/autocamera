from __common_imports__ import *

class hfs_bag_processor:
    """!
        This program will read the bag files from our human factors study, extract the joint angles
        and compute the position of the end-effectors for all arms. The joint angles nad end-effector
        positions will be written to a new bag file. This would also remove any image data in the files.
    """
    
    def __init__(self):
        """!
            The initialization function for the hfs_bag_processor class.
        """
        rospy.init_node('bag_processor')
        
        # Defining the kinematic computation variables
        self.__ecm_robot__ = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        self.__ecm_kin__ = KDLKinematics(self.__ecm_robot__, self.__ecm_robot__.links[0].name, self.__ecm_robot__.links[-1].name)
        
        self.__psm1_robot__ = URDF.from_parameter_server('/dvrk_psm1/robot_description')
        self.__psm1_kin__ = KDLKinematics(self.__psm1_robot__, self.__psm1_robot__.links[0].name, self.__psm1_robot__.links[-1].name)
        
        self.__psm2_robot__ = URDF.from_parameter_server('/dvrk_psm2/robot_description')
        self.__psm2_kin__ = KDLKinematics(self.__psm2_robot__, self.__psm2_robot__.links[0].name, self.__psm2_robot__.links[-1].name)
        
        self.__mtml_robot__ = URDF.from_parameter_server('/dvrk_mtml/robot_description')
        self.__mtml_kin__ = KDLKinematics(self.__mtml_robot__, self.__mtml_robot__.links[0].name, self.__mtml_robot__.links[-1].name)
        
        self.__mtmr_robot__ = URDF.from_parameter_server('/dvrk_mtmr/robot_description')
        self.__mtmr_kin__ = KDLKinematics(self.__mtmr_robot__, self.__mtmr_robot__.links[0].name, self.__mtmr_robot__.links[-1].name)
        
        self.topics = {'mtml': '/dvrk/MTML/state_joint_current',
                       'mtmr': '/dvrk/MTMR/state_joint_current',
                       'psm1': '/dvrk/PSM1/state_joint_current',
                       'psm2': '/dvrk/PSM2/state_joint_current',
                       'ecm' : '/dvrk/ECM/state_joint_current',
                       'camera':'/dvrk/footpedals/camera',
                       'clutch':'/dvrk/footpedals/clutch',
                       'headsensor':'/dvrk/footpedals/coag'}
        
        self.kin = {'ecm':self.__ecm_kin__,
                    'psm1':self.__psm1_kin__,
                    'psm2':self.__psm2_kin__,
                    'mtml':self.__mtml_kin__,
                    'mtmr':self.__mtmr_kin__}
    def spin(self):
        rospy.spin()
                
    def process_folder(self, folder_with_bag_files):
        """!
            @param folder_with_bag_files : The location of a folder that contains all the bag
            files we are going to prcess
        """
        
        # Find all the .bag files in folder
        items = os.listdir(folder_with_bag_files)
        bag_items = []
        for item in items:
            if item.endswith(".bag"):
                bag_items.append(item) 
        
        # Process the bag files in the folder
        # and save to new bag file
        for bag_file_name in bag_items:
            output_file_name = bag_file_name[0:-4] + "_processd.bag"
            self.process_bag_file(folder_with_bag_files, bag_file_name, output_file_name)
        
    def process_bag_file(self, folder, bag_file_name, output_file_name):
        """!
            Processes a bag file. Reads the file, computes the end-effector
            positions and writes the relevant data to a new bag file.
            
            @param folder : The folder where the bag files are in
            @param bag_file_name : The name (and location) of the bag file to be processed
            @param output_file_name : The name of the output bag file
        """
        print('input bag file name = {}\noutput bag file name = {}\n\n'.format(bag_file_name, output_file_name))

        bag_file = rosbag.Bag(folder + '/'+bag_file_name)
        output_bag_file = rosbag.Bag(folder+'/processed/'+output_file_name, 'w')
         
        [topic, messages, times] = self.compute_fkine(bag_file, 'mtml')
        self.save_to_bag(output_bag_file, topic, messages, times)
         
        [topic, messages, times] = self.compute_fkine(bag_file, 'mtmr')
        self.save_to_bag(output_bag_file, topic, messages, times)
         
        [topic, messages, times] = self.compute_fkine(bag_file, 'psm1')
        self.save_to_bag(output_bag_file, topic, messages, times)
         
        [topic, messages, times] = self.compute_fkine(bag_file, 'psm2')
        self.save_to_bag(output_bag_file, topic, messages, times)
        
        [topic, messages, times] = self.compute_fkine(bag_file, 'ecm')
        self.save_to_bag(output_bag_file, topic, messages, times)
        
        # TODO : Write the clutches to the new bag file without modification
        
        bag_file.close()
        output_bag_file.close()
        
    def save_to_bag(self, output_bag_file, topic, messages, times):
        """!
            This function saves the data to a bag file.
            
            @param output_bag_file : The bag file to be written to
            @param topic : The topic
            @param messages : A list of messages
            @param times : A list of timestamps for corresponding messages
        """
        
        for i in range(len(times)):
            output_bag_file.write(topic, messages[i], t=times[i])
            
        
    def compute_fkine(self, bag_file, arm_name):
        """!
            Compute the forward kinematics of the given arm.
            Returns list of messages and timestamps
            
            @param bag_file : The bag_file to use
            @param arm_name : The name of the arm (MTML, MTMR, PSM1, PSM2, ECM)
            
            @return topic : The topic name
            @return messages : List of position messages
            @return times : List of timestamps
        """
        bag_messages = bag_file.read_messages( topics=[self.topics[arm_name]])
        
        messages = []
        times = []
        for topic, msg, time_current_message in bag_messages:
            p = Pose()
            T = []
            if arm_name.lower() != 'ecm':
                T = self.kin[arm_name].forward(msg.position[0:-1])
            else:
                print(arm_name)
                print(msg.position)
                T = self.kin[arm_name].forward(msg.position)
                
            q = PoseConv.to_pos_quat(T)
            p.position.x = q[0][0]
            p.position.y = q[0][1]
            p.position.z = q[0][2]
            
            p.orientation.w = q[1][0]
            p.orientation.x = q[1][1]
            p.orientation.y = q[1][2]
            p.orientation.z = q[1][3]
            
            messages.append(p)
            times.append(time_current_message)
            
        return self.topics[arm_name], messages, times
    
if __name__ == "__main__":
    hfs = hfs_bag_processor()
    hfs.process_folder('/home/dvrk/Downloads/hfs_recordings/subject1')
    hfs.spin()
    