# Exercise 1 - Display an image of the camera feed to the screen
import rclpy
from rclpy.node import Node
import cv2
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.exceptions import ROSInterruptException
import signal

class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.move_cmd = Twist()
        self.target_detected = False

    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        # Show the resultant images you have created.

        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
    
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_blue_lower = np.array([120- self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120+ self.sensitivity, 255, 255])
        
        blue_image = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)

        contours, hierarchy = cv2.findContours(blue_image,mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
        if len(contours) > 0:
            
            # Loop over the contours
            # There are a few different methods for identifying which contour is the biggest:
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            c = max(contours, key=cv2.contourArea)
            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(c) > 100: #<What do you think is a suitable area?>
                            #Moments can calculate the center of the contour
                M = cv2.moments(c)
                
                if M['m00'] == 0:
                    return
                
                cx = int(M['m10']/M['m00'])
                width = image.shape[1]
                error = cx - width // 2
                # draw a circle on the contour you're identifying
                
                if cv2.contourArea(c)>1000:
                    self.move.cmd.linear.x =0.0
                    self.move.cmd.angular.z =0.0
                    self.target_detected = True
                    print("BLUE BOX WITHIN ONE METER, STOPPING")
                    
                else:
                    self.move.cmd.linear.x =0.15
                    self.move.cmd.angular.z = -float(error)/200       
                    ("APPROACHING BLUE BOX")
            
        else:
            self.move.cmd.linear.x =0.0
            self.move.cmd.angular.z = 0.2      
        
        self.publisher.publish(self.move_cmd)

        cv2.imshow('threshold_Feed2', image)
        cv2.waitKey(3)
        
        
        
def main():

    rclpy.init(args=None)
    cI = colourIdentifier()
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
    # Remember to destroy all image windows before closing node
    

    try:
        rclpy.spin(cI)
            
    except ROSInterruptException:
        pass
    
    finally:
        cI.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
# Check if the node is executing in the main path
if __name__ == '__main__':
    main()