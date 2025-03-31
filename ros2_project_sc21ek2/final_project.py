import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav2_msgs.action import NavigateToPose
from math import sin, cos
import numpy as np
import time

class RGBNavigator(Node):
    def __init__(self):
        super().__init__('rgb_nav')
        
        self.sensitivity = 10 #hsv colour filtering
        self.goal_active = False
        
        #navigate action client and camera
        self.nav_client = ActionClient(self, NavigateToPose,'navigate_to_pose')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 30)
        self.bridge = CvBridge()
    
        #List of coordinates        
        self.target_coords = [
            (-3.9, -1.65, 3.14),
            (3.07,-7.35,0.0),
            (-3.08,-9.16,3.14),
            
        ]
        self.index = 0
        self.timer = self.create_timer(1.0,self.timer_callback) #trigger goal send
    
    
    def send_goal(self,x,y,yaw): #sends goasl to nav2
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
            
    def timer_callback(self):  #timer to manage goal progression
        if self.goal_active:
            return
        if self.index <len(self.target_coords):
            x,y, yaw = self.target_coords[self.index]
            self.send_goal(x,y,yaw)
            self.goal_active= True
                 
    def goal_response_callback(self, future):  #cllback when goal is rejected
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted') #cllback when goal is accepted
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.goal_active = False

    def feedback_callback(self, feedback_msg):
            feedback = feedback_msg.feedback

    def callback(self, data):
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
            colours_hsv = {
                'red_image': cv2.inRange(Hsv_image, np.array([0 - self.sensitivity, 100, 100]), np.array([0 + self.sensitivity, 255, 255])),
                'green_image': cv2.inRange(Hsv_image, np.array([60 - self.sensitivity, 100, 100]), np.array([60 + self.sensitivity, 255, 255])),
                'blue_image': cv2.inRange(Hsv_image, np.array([110 - self.sensitivity, 100, 50]), np.array([130 + self.sensitivity, 255, 255]))

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
    
            cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
            cv2.imshow('camera_Feed', image)
            cv2.resizeWindow('camera_Feed',320,240)
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
if __name__ == '__main__':
    main()

