#This is a class to process Bag classes for either MATLAB file output or Tensorflow purposes
#usage:		
#c = ConvertBags('random_movements_2017-03-23-12-14-08.bag')
#c.WriteJointAngleFile('test_j.txt');
#c.WriteCartesianFile('test_c.txt');
# note: it doesn't check if the file exists

import rosbag
import os.path
import numpy 
from geometry_msgs.msg._PoseStamped import PoseStamped
from hrl_geom.pose_converter import PoseConv
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics

class ConvertBags:
        bag_file_name=[];
	bag=[]  # bag file for conversion
 	

	# topic definitions...these should be modified if the topics to be written change.  TOdo: Write filler code.
        topics_cartesian =['/dvrk/PSM1/position_cartesian_current', 
			'/dvrk/PSM2/position_cartesian_current', '/dvrk/ECM/position_cartesian_current']
	topics_joint =['/dvrk/PSM1/state_joint_current', '/dvrk/PSM2/state_joint_current', '/dvrk/ECM/state_joint_current']
	
	#this are the characters append to each of the topics for the first line of the output files.
        cartesian_header = "xyzabcd"  	
	joint_header = "1234567"

	def __init__(self, bag_file_name):
 	#Constructor for bag object
	###################################################	

                self.bag_file_name = bag_file_name
		if (os.path.exists(bag_file_name)):
			print ("processing " +bag_file_name +"\n")
		else:
			print (bag_file_name + " doesn't exist\n")
            
    

        def ComputeCartesian(self, new_bag_name):
            self.ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
            self.ecm_kin = KDLKinematics(self.ecm_robot, self.ecm_robot.links[0].name, self.ecm_robot.links[-1].name)
            
            self.psm1_robot = URDF.from_parameter_server('/dvrk_psm1/robot_description')
            self.psm1_kin = KDLKinematics(self.psm1_robot, self.psm1_robot.links[0].name, self.psm1_robot.links[-1].name)
            
            self.psm2_robot = URDF.from_parameter_server('/dvrk_psm2/robot_description')
            self.psm2_kin = KDLKinematics(self.psm2_robot, self.psm2_robot.links[0].name, self.psm2_robot.links[-1].name)
            
            my_bag = rosbag.Bag(self.bag_file_name)
            new_bag = rosbag.Bag(new_bag_name, 'w')
            ecm_last_angles = [0,0,0,0]
            for topic, msg, t in my_bag.read_messages(self.topics_joint):
                if "/dvrk/ECM/state_joint_current" in topic > 0:
                    ecm_last_angles = msg.position
                    new_topic = "/dvrk/ECM/position_cartesian_current"
                    new_pose = self.ecm_kin.forward(msg.position)
                    
                    new_msg = PoseStamped()
                    quaterion_pose = PoseConv.to_pos_quat(new_pose)
                    new_msg.pose.position.x = quaterion_pose[0][0]
                    new_msg.pose.position.y = quaterion_pose[0][1]
                    new_msg.pose.position.z =  quaterion_pose[0][2]
                    
                    new_msg.pose.orientation.x =  quaterion_pose[1][0]
                    new_msg.pose.orientation.y =  quaterion_pose[1][1]
                    new_msg.pose.orientation.z =  quaterion_pose[1][2]
                    new_msg.pose.orientation.w =  quaterion_pose[1][3]
                    
                    new_bag.write(new_topic, new_msg, t)
                    new_bag.write(topic, msg, t)
                    
                if "/dvrk/PSM1/state_joint_current" in topic > 0:
                    new_topic = "/dvrk/PSM1/position_cartesian_current"
                    ecm_inv =  self.ecm_kin.forward(ecm_last_angles)**-1
                    new_pose = ecm_inv * self.psm1_kin.forward(msg.position[0:-1])
                    
                    new_msg = PoseStamped()
                    quaterion_pose = PoseConv.to_pos_quat(new_pose)
                    new_msg.pose.position.x = quaterion_pose[0][0]
                    new_msg.pose.position.y = quaterion_pose[0][1]
                    new_msg.pose.position.z =  quaterion_pose[0][2]
                    
                    new_msg.pose.orientation.x =  quaterion_pose[1][0]
                    new_msg.pose.orientation.y =  quaterion_pose[1][1]
                    new_msg.pose.orientation.z =  quaterion_pose[1][2]
                    new_msg.pose.orientation.w =  quaterion_pose[1][3]
                    
                    new_bag.write(new_topic, new_msg, t)
                    new_bag.write(topic, msg, t)
                    
                if "/dvrk/PSM2/state_joint_current" in topic > 0:
                    new_topic = "/dvrk/PSM2/position_cartesian_current"
                    ecm_inv =  self.ecm_kin.forward(ecm_last_angles)**-1
                    new_pose = ecm_inv * self.psm2_kin.forward(msg.position[0:-1])
                    
                    new_msg = PoseStamped()
                    quaterion_pose = PoseConv.to_pos_quat(new_pose)
                    new_msg.pose.position.x = quaterion_pose[0][0]
                    new_msg.pose.position.y = quaterion_pose[0][1]
                    new_msg.pose.position.z =  quaterion_pose[0][2]
                    
                    new_msg.pose.orientation.x =  quaterion_pose[1][0]
                    new_msg.pose.orientation.y =  quaterion_pose[1][1]
                    new_msg.pose.orientation.z =  quaterion_pose[1][2]
                    new_msg.pose.orientation.w =  quaterion_pose[1][3]
                    
                    new_bag.write(new_topic, new_msg, t)
                    new_bag.write(topic, msg, t)
                    
            my_bag.close();
            new_bag.close();
             
	def WriteCartesianFile(self, filename):
	#Write only the cartesian topics from the bag files to a text file for MATLAB NN toolkit.
	###################################################	

		#open both files
		fid = open(filename, "w") 
		self.bag = rosbag.Bag(self.bag_file_name)
                
		print ("MatlabCartesian writing :" + filename + "from " + self.bag_file_name);
                #Traverse the bag file and write information to the CartesianFile.		
		for topic, msg, t in self.bag.read_messages(self.topics_cartesian):
			if topic.find("position_cartesian_current") > 0:
				fid.write(topic)
				fid.write("\n")
	                	fid.write("{} {} {} {} {} {} {}\n".format (msg.pose.position.x, msg.pose.position.y, 					msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, 					msg.pose.orientation.w))
		#close both files		
		fid.close
                self.bag.close(); 
		self.bag =[]; 


	def WriteJointAngleFile(self, filename):
	#Write the joint angles from the bag files to a text file for MATLAB NN input	
	###################################################	

		#open both files
		fid = open(filename, "w") 
		self.bag = rosbag.Bag(self.bag_file_name)

		print ("MatlabJointAngle:writing :" + filename+ " from==>" + self.bag_file_name)
                #Traverse the bag file and write information to the CartesianFile.		
		for topic, msg, t in self.bag.read_messages(self.topics_joint):		
			if topic.find("state_joint_current") > 0:  
				fid.write(topic + "\n")
				
                		for x in msg.position:
					fid.write(str(x) +" ")
                         	fid.write("\n"); 
		fid.close
                self.bag.close();  # Reset to the beginning
		self.bag =[];

	def LoadTensorFileJoint(self):
	###################################################
	# returns database variable with all the data and a count.	
		#open bag file
		self.bag = rosbag.Bag(self.bag_file_name)
                #assign numpy arrays for each of the topics

		d = {}; # create a dictionary that contains a set of data for each topic.
                for topic in self.topics_joint:
			count = self.bag.get_message_count(topic)
			if topic.find("ECM") >0:
                                #print ("ECM count =" + str(count))
				d[topic] = numpy.empty([count, 4])
			else:
				d[topic] = numpy.empty([count, 7])
                        i = 0
			for top, msg, t in self.bag.read_messages(topic):
				j = 0
				for value in msg.position:
					d[topic][i][j] = value
					j = j+1
				i = i + 1
		return (d, count)


	def WriteTensorFileJoint(self, filename):
 	#Constructor for bag object
	###################################################	
       	# Write a tensor flow file of joint angles to a file.

		print ("TensorFileJoint: writing :" + filename+ " from==>" + self.bag_file_name)
                #load a dictionary file of all the joint data.
                data, count = self.LoadTensorFileJoint()
                #print (str(count))
		#open file to be written.
		fid = open(filename, "w") 
		

		#write out the first column of the file
		s = "";
 		for topic in self.topics_joint:
			for c in self.joint_header:
				s = s+ topic + "_" + c + ","
 		s = s[0:-1]  # remove the last comma

                #write a header line
                fid.write (s+"\n")
		
                #print ("header line:" + s + "\n")
                count_topics = len (self.topics_joint)
		count_items= len (self.joint_header)
 				
		#assuming the same size messages for all topics.
		
		for i in range (0, count-1):
			line ="";
			j = 0;
                	for topic in self.topics_joint:
				if topic.find("ECM") >0: 
					jcount = 4
				else:
					jcount = 7

				for j in range (0,jcount):		
					line = line + str(data[topic][i][j]) +","
		
			line = line[0:-1] # remove the last comma
			fid.write(line +"\n")
				
				
		fid.close
               
	def LoadTensorFileCartesian(self):
	###################################################
	# returns database variable with all the data and a count.	

		#open bag file
		self.bag = rosbag.Bag(self.bag_file_name)
                #assign numpy arrays for each of the topics

		d = {}; # create a dictionary that contains a set of data for each topic.

		#how many lines of data?
		
                for topic in self.topics_cartesian:
			count = self.bag.get_message_count(topic)
                        #print (topic + " " + str (count) +"\n")
			d[topic] = numpy.empty([count, 7])
                        i = 0;
			for each_topic, msg, t in self.bag.read_messages(topic):
				d[each_topic][i][:] = numpy.array([msg.pose.position.x, msg.pose.position.y, 						 	 msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, 								 msg.pose.orientation.z, msg.pose.orientation.w])
				i = i + 1

		return (d, count)

	def WriteTensorFileCartesian(self, filename):
	###################################################	
        # Write a tensor flow file of Cartesian (xyz and quarternions) to a file.
		print ("TensorFileJoint: writing :" + filename+ " from==> " + self.bag_file_name)
                #load a dictionary file of all the joint data.
                data, count = self.LoadTensorFileCartesian()
                #print ("read the file with count "+ str(count) +"\n");
                
		#open file to be written.
		fid = open(filename, "w") 
		

		#write out the first column of the file
		s = "";
 		for topic in self.topics_cartesian:
			for c in self.cartesian_header:
				s = s+ topic + "_" + c + ","
 		s = s[0:-1]  # remove the last comma

                #write a header line
                fid.write (s+"\n")
		 
                #print ("header line:" + s + "\n")
                count_topics = len (self.topics_cartesian)
		count_items= len (self.cartesian_header)
 		
		
		#assuming the same size messages for all topics.
		
		for i in range (0, count-1):
			line ="";
                	for topic in self.topics_cartesian:
				for j in range (0, count_items):		
					line = line + str(data[topic][i][j]) +","
		
			line = line[0:-1] # remove the last comma
			fid.write(line + "\n")
				
		fid.close


filename = ''
#usage:	
a = ConvertBags('peg_transfer_with_autocamera.bag')
a.ComputeCartesian('new_peg_transfer_with_autocamera.bag')
a = ConvertBags('new_peg_transfer_with_autocamera.bag')
 
# a.WriteTensorFileCartesian('peg_transfer_with_autocamera_tensor_cartesian.txt')
# a.WriteTensorFileJoint('peg_transfer_with_autocamera_tensor_joint.txt')
a.WriteCartesianFile('peg_transfer_with_autocamera_matlab_cartesian.txt')
a.WriteJointAngleFile('peg_transfer_with_autocamera_matlab_joint.txt')
# 	
# c = ConvertBags('random_movements_2017-03-23-12-14-08.bag')
# c.ComputeCartesian('new_random_movements_2017-03-23-12-14-08.bag')
# c = ConvertBags('new_random_movements_2017-03-23-12-14-08.bag')
# c.WriteTensorFileCartesian('random_movements_tensor_cartesian.txt')
# c.WriteTensorFileJoint('random_movements_tensor_joint.txt')
# c.WriteCartesianFile('random_movements_matlab_cartesian.txt')
# c.WriteJointAngleFile('random_movements_matlab_joint.txt')
# # 
# b = ConvertBags('peg_transfer_2_2017-03-23-12-08-09.bag')
# b.ComputeCartesian('new_peg_transfer_2_2017-03-23-12-08-09.bag')
# b = ConvertBags('new_peg_transfer_2_2017-03-23-12-08-09.bag')
# b.WriteTensorFileCartesian('peg_transfer_tensor_cartesian.txt')
# b.WriteTensorFileJoint('peg_transfer_tensor_joint.txt')
# b.WriteCartesianFile('peg_transfer_matlab_cartesian.txt')
# b.WriteJointAngleFile('peg_transfer_matlab_joint.txt')
# # 
# d = ConvertBags('luke_joy_2017-04-07-16-05-49.bag')
# d.ComputeCartesian('new_luke_joy_2017-04-07-16-05-49.bag')
# d = ConvertBags('new_luke_joy_2017-04-07-16-05-49.bag')
# d.WriteTensorFileCartesian('luke_joy_tensor_cartesian.txt')
# d.WriteTensorFileJoint('luke_joy_tensor_joint.txt')
# d.WriteCartesianFile('luke_joy_matlab_cartesian.txt')
# d.WriteJointAngleFile('luke_joy_matlab_joint.txt')
#   
# d = ConvertBags('luke_peg_2017-04-07-16-11-56_2017-04-07-16-14-47.bag')
# d.ComputeCartesian('new_luke_peg_2017-04-07-16-11-56_2017-04-07-16-14-47.bag')
# d = ConvertBags('new_luke_peg_2017-04-07-16-11-56_2017-04-07-16-14-47.bag')
# d.WriteTensorFileCartesian('luke_peg_tensor_cartesian.txt')
# d.WriteTensorFileJoint('luke_peg_tensor_joint.txt')
# d.WriteCartesianFile('luke_peg_matlab_cartesian.txt')
# d.WriteJointAngleFile('luke_peg_matlab_joint.txt')


