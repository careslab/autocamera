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
            self.process_bag_file(bag_file_name, output_file_name)
        
    def process_bag_file(self, bag_file_name, output_file_name):
        """!
            Processes a bag file. Reads the file, computes the end-effector
            positions and writes the relevant data to a new bag file.
            
            @param bag_file_name : The name (and location) of the bag file to be processed
            @param output_file_name : The name of the output bag file
        """
        pass
    
    def compute_fkine(self, bag_file, arm_name):
        """!
            Compute the forward kinematics of the given arm.
            Returns list of messages and timestamps
            
            @param bag_file : The bag_file to use
            @param arm_name : The name of the arm (MTML, MTMR, PSM1, PSM2, ECM)
            
            @return messages : List of position messages
            @return times : List of timestamps
        """
        pass
    
    