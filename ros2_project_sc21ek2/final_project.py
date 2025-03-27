# COMMANDS
# 
# ~/ros2_ws $ ros2 launch turtlebot3_navigation2 navigation2.ros2 run ros2_project_sc21ek2 final_project
# launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/ros2_project_sc21ek2/map/map.yaml
# ros2 launch turtlebot3_gazebo turtlebot3_task_world_2025.launch.py
# source ~/.bashrc


# Exercise 1 - Display an image of the camera feed to the screen
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from rclpy.action import ActionClient

import cv2
import numpy as np
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
import signal
from nav2_msgs.action import NavigateToPose
import random

class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 30)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.move_cmd = Twist()
        self.target_detected = False
        
        self.nav_client = ActionClient(self, NavigateToPose,'navigate_to_pose')
        self.goal_running = False
        self.timer = self.create_timer(10.0, self.send_random_goal)
        
    def send_random_goal(self):
        if self.target_detected or self.goal_running:
            return
        
        x = random.uniform(0.0,0.0)
        y = random.uniform(0.0,0.0)
    
        
        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        self.goal_running=True
        self.nav_client.send_goal_async(goal).add_done_callback(self.get_result_callback)
        
        self.get_logger().info("sending goal")
        
    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)
       
    def get_result_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Navigation result: {result}') 
        except Exception as e:
                self.get_logger().error(f'{e}') 

        self.goal_running = False

    def image_callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        # Show the resultant images you have created.

        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
    
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_blue_lower = np.array([120- 10, 100, 100])
        hsv_blue_upper = np.array([120+ 10, 255, 255])
        
        blue_image = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)

        contours, _ = cv2.findContours(blue_image,mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
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
                    self.move_cmd.linear.x =0.0
                    self.move_cmd.angular.z =0.0
                    self.target_detected = True
                    self.get_logger().info("BLUE BOX WITHIN ONE METER, STOPPING")
                    
                else:
                    self.move_cmd.linear.x =0.15
                    self.move_cmd.angular.z = -float(error)/200       
                    self.get_logger().info("APPROACHING BLUE BOX")
            
        else:
            self.move_cmd.linear.x =0.0
            self.move_cmd.angular.z = 0.2      
        
        self.publisher.publish(self.move_cmd)
        
        filtered = cv2.bitwise_and(image,image,mask=blue_image)
        cv2.namedWindow("camera_Feed",cv2.WINDOW_NORMAL)
        

        cv2.imshow('camera_Feed', filtered)
        cv2.waitKey(3)
        
        
        
def main():

    rclpy.init(args=None)
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
    # Remember to destroy all image windows before closing node
    

    try:
        node = colourIdentifier()
        rclpy.spin(node)
            
    except Exception as e:
        print (f"exception: {e}")
        pass
    
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
            cv2.destroyAllWindows()
# Check if the node is executing in the main path
if __name__ == '__main__':
    main()