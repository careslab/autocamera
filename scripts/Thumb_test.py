import rospy
import Leap, sys, thread, time

class MyListener(Leap.Listener):

    def on_init(self, controller):
	print "Initialized"

    def on_connect(self, controller):
        print "Connected"

    def on_frame(self, controller):
        # Get the most recent frame
        frame = controller.frame()
	# Get thumb
	for hand in frame.hands:
	    wrist = hand.wrist_position
	    if hand.is_right:
		print 'Right wrist postion %s'%(wrist)
	        #for finger in hand.fingers:
		 #   if finger.type == 2:
		  #      print 'Right MF %s'%(finger.tip_position[1]) 
	    #if hand.is_left:
		#print 'Left Hand Rotation %s'%(hand.rotation_angle(controller.frame(5),hand.basis.z_basis))
		#for finger in hand.fingers:
		 #   if finger.type == 2:
		   #     print 'Left MF %s'%(finger.tip_position[1])

def main():
    # Create a sample listener and controller
    listener = MyListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()
