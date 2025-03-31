# COMMANDS
# 
# ~/ros2_ws $ ros2 launch turtlebot3_navigation2 navigation2.ros2 run ros2_project_sc21ek2 final_project
# launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/ros2_project_sc21ek2/map/map.yaml
# ros2 launch turtlebot3_gazebo turtlebot3_task_world_2025.launch.py
# source ~/.bashrc


# Exercise 1 - Display an image of the camera feed to the screen
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from math import sin, cos

import cv2
import numpy as np
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
import signal
from nav2_msgs.action import NavigateToPose
import random

class RGBNavigator(Node):
    def __init__(self):
        super().__init__('rgb_nav')
        
        self.sensitivity = 10

        self.nav_client = ActionClient(self, NavigateToPose,'navigate_to_pose')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 30)
        self.bridge = CvBridge()
        
        self.found_red = False;
        self.found_green = False;
        self.found_blue = False;
        self.reach_blue = False;
        
        self.target_coords = [
            (-2.18, -4.17, 0.00256),
            (1.64,-7.46,0.00641),
            (0.119,-10.8,0.00641)
        ]
        
        self.index = 0
        
        self.timer = self.create_timer(2.0, self.timer_callback)
    
    
    def send_goal(self,x,y,yaw):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            goal_msg.pose.pose.orientation.z = sin(yaw / 2)
            goal_msg.pose.pose.orientation.w = cos(yaw / 2)
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

            self.nav_client.wait_for_server()
            self.send_goal_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

            self.send_goal_future.add_done_callback(self.goal_response_callback)
            
    def timer_callback(self):
        if not all ([self.found_red, self.found_green, self.found_blue]):
            if self.index <len(self.target_coords):
                x,y, yaw = self.target_coords[self.index]
                self.send_goal(x,y,yaw)
                self.get_logger().info('Goal sent')
                self.index+=1
            else:
                self.get_logger().info('not all colours Detected ')

        elif self.found_blue and not self.reach_blue:
                self.get_logger().info('moving towards blue box')
                x,y, yaw = self.target_coords[2]
                self.send_goal(x,y,yaw)
                
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')


    def feedback_callback(self, feedback_msg):
            feedback = feedback_msg.feedback
            print(feedback)           

        
    def callback(self, data):
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
            cv2.imshow('camera_Feed', image)
            cv2.resizeWindow('camera_Feed',320,240)
            cv2.waitKey(3)
            
            colours_hsv = {
                'red_image': cv2.inRange(Hsv_image, np.array([0 - self.sensitivity, 100, 100]), np.array([0 + self.sensitivity, 255, 255])),
                'green_image': cv2.inRange(Hsv_image, np.array([60 - self.sensitivity, 100, 100]), np.array([60 + self.sensitivity, 255, 255])),
                'blue_image': cv2.inRange(Hsv_image, np.array([100 - self.sensitivity, 150, 50]), np.array([100 + self.sensitivity, 255, 255])),



            }
            
            for colour, colourhsv in colours_hsv.items():
                contours,_ = cv2.findContours(colourhsv,mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
                for c in contours:
                    if cv2.contourArea(c) > 100: 
                            M = cv2.moments(c)
                            if M['m00'] != 0:
                                cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                                (x, y), radius = cv2.minEnclosingCircle(c)
                                center = (int(x),int(y)) 
                                radius = int(radius) 
                                cv2.circle(image,center,radius,(255,255,0) ,1)
                                
                                if colour == 'red' and not self.found_red:
                                    self.found_red = True
                                    self.get_logger().info('red detected')

                                
                                if colour == 'green' and not self.found_green:
                                    self.found_green = True
                                    self.get_logger().info('green detected')

                                    
                                if colour == 'blue' and not self.found_blue:
                                    self.found_blue = True
                                    self.get_logger().info('blue detected')
    
            cv2.imshow('Colour DEtected', image)
            cv2.waitKey(3)


def main():

    rclpy.init(args=None)
    node = RGBNavigator()
    try:
        rclpy.spin(node)        
    except KeyboardInterrupt:
        pass
    finally:
            node.destroy_node()
            rclpy.shutdown()
            cv2.destroyAllWindows()
# Check if the node is executing in the main path
if __name__ == '__main__':
    main()

