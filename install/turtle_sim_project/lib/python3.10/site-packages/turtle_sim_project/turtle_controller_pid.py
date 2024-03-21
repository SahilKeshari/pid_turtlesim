#! /usr/bin/env python3
import rclpy
import math
from rclpy.node import Node 

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("TurtleControllerNode")
        self.pose_ = None
        self.pose_subscriber_ = self.create_subscription(Pose,'turtle1/pose',self.callback_pose,10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist,'turtle1/cmd_vel',10)
        self.control_loop_timer_ = self.create_timer(0.01,self.control_loop)
        self.target_x,self.target_y = 8.0,8.0

        self.Kp = 2  # Proportional gain
        self.Ki = 0.0  # Integral gain
        self.Kd = 0.1  # Derivative gain

        # Variables for PID control
        self.prev_error = 0.0
        self.integral = 0.0

        # Setpoint (desired position)
        # self.target_x = 5.0
        # self.target_y = 5.0

        # Current pose
        self.pose_ = None


    def callback_pose(self,msg):
        self.pose_ = msg
    
    def control_loop(self):
        if self.pose_ == None:
            return
        
        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance = (math.sqrt(dist_x * dist_x + dist_y * dist_y))

        # PID control
        P = self.Kp * distance

        self.integral += distance
        I = self.Ki * self.integral

        derivative = distance - self.prev_error
        D = self.Kd * derivative

        output = P + I + D

        self.prev_error = distance

        msg = Twist()

        if distance > 0.1:
            # Position control
            # msg.linear.x = min(max(output, -1.0), 1.0)  # Limit velocity to [-1, 1]
            msg.linear.x = min(max(output, -2*distance), 2*distance)

            # Orientation control
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            msg.angular.z = 6 * diff


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
