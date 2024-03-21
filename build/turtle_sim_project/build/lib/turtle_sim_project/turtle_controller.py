#! /usr/bin/env python3
import rclpy
import math
from rclpy.node import Node 

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import random

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("TurtleControllerNode")
        self.pose_ = None
        self.pose_subscriber_ = self.create_subscription(Pose,'turtle1/pose',self.callback_pose,10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist,'turtle1/cmd_vel',10)
        self.control_loop_timer_ = self.create_timer(0.01,self.control_loop)
        self.target_x = random.uniform(0.0,10.0)
        self.target_y = random.uniform(0.0,10.0)


    def callback_pose(self,msg):
        self.pose_ = msg
    
    def control_loop(self):
        if self.pose_ == None:
            return
        
        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance = (math.sqrt(dist_x * dist_x + dist_y * dist_y))

        msg = Twist()

        if distance > 0.5:
            #position
            msg.linear.x = 2*distance

            #orientation 
            goal_theta = math.atan2(dist_y,dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi

            msg.angular.z = 6*diff 


        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
