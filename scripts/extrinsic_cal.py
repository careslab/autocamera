# import the necessary packages
import argparse
import cv2
 
 
def click(event, x, y, flags, param):
	if event == cv2.EVENT_LBUTTONDOWN:
		refPt = [(x, y)]
		cv2.circle(image, (x, y), 2, (0,255,0), -1)
		ecm_read_cb.save = True
 		print((x,y).__str__())
 
	# check to see if the left mouse button was released
	elif event == cv2.EVENT_LBUTTONUP:
		# record the ending (x, y) coordinates and indicate that
		cv2.imshow("image", image)

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Path to the image")
args = vars(ap.parse_args())

ecm_joint_angles = []
def ecm_read_cb(msg):
    if ecm_read_cb.save == True:
        global ecm_joint_angles, ecm_robot, ecm_kin
        rospy.loginfo("ecm joint angles : " + msg.position.__str__())
        ecm_read_cb.save = False
        p, _ = ecm_kin.FK(msg.position)
        ecm_joint_angles.append(msg.position)
        
        ecm_read_cb.count += 1
ecm_read_cb.save = False
ecm_read_cb.count = 0
 
def main():
    global save, ecm_robot, ecm_kin
    
    rospy.init_node('ecm_extrinsic_calibration')
    # Get the joint angles from the hardware and move the simulation from hardware
    rospy.Subscriber('/dvrk/ECM/position_joint_current', JointState, ecm_read_cb)
    
    if ecm_robot is None:
        ecm_robot = URDF.from_parameter_server('/dvrk_ecm/robot_description')
        ecm_kin = KDLKinematics(ecm_robot, ecm_robot.links[0].name, ecm_robot.links[-1].name)
            
    
    # load the image, clone it, and setup the mouse callback function
	image = cv2.imread(args["image"])
	clone = image.copy()
	cv2.namedWindow("image")
	cv2.setMouseCallback("image", click)
	 
	# keep looping until the 'c' key is pressed
	while True:
		# display the image and wait for a keypress
		cv2.imshow("image", image)
		key = cv2.waitKey(1) & 0xFF
	 
		# if the 'r' key is pressed, reset the cropping region
		if key == ord("r"):
			image = clone.copy()
	 
		# if the 'c' key is pressed, break from the loop
		elif key == ord("c"):
			break
	 
	# if there are two reference points, then crop the region of interest
	# from teh image and display it
	if len(refPt) == 2:
		roi = clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
		cv2.imshow("ROI", roi)
		cv2.waitKey(0)
	 
	# close all open windows
	cv2.destroyAllWindows()

    rospy.spin()
    
            
    
if __name__ == "__main__":
    main()
