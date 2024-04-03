#! /usr/bin/env python3
import rclpy
import math
from rclpy.node import Node 

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from custom_msgs.msg import Movement

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("TurtleControllerNode")
        self.pose_ = None
        self.pose_subscriber_ = self.create_subscription(Pose,'turtle1/pose',self.callback_pose,10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist,'turtle1/cmd_vel',10)
        self.control_loop_timer_ = self.create_timer(0.01,self.control_loop)
        self.target_x,self.target_y = 1.0,1.0

        self.Kp = 2.0  # Proportional gain
        self.Ki = 0.0  # Integral gain
        self.Kd = 0.1  # Derivative gain

        # Variables for PID control
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_error_orie = 0.0

        self.pose_ = None

        self.prev_linear_velocity = 0.0
        self.prev_angular_velocity = 0.0

        # maximum acceleration and deceleration limits
        self.max_acceleration = 2.0
        self.max_deceleration = -2.0

        self.flag_moveup = False

        self.desired_position = False
        self.desired_orientation = False

        self.plot_movement = self.create_publisher(Movement,'turtle1/movement_data',10)

    def callback_pose(self,msg):
        global theta
        self.pose_ = msg
        theta = self.pose_.theta
    
    def control_loop(self):
        if self.target_x > 10.0 or self.target_y > 10.0:
            rclpy.shutdown()

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

        if not self.desired_position:
            if distance > 0.1:

                desired_linear_velocity = min(max(output, -2*distance), 2*distance)
                desired_angular_velocity = math.atan2(dist_y, dist_x)

                # Calculate velocity changes
                delta_linear_velocity = desired_linear_velocity - self.prev_linear_velocity
                delta_angular_velocity = desired_angular_velocity - self.prev_angular_velocity

                delta_linear_velocity = self.limit_velocity_change(delta_linear_velocity)
                delta_angular_velocity = self.limit_velocity_change(delta_angular_velocity)
                
                # Apply acceleration and deceleration limits
                desired_linear_velocity = self.prev_linear_velocity + delta_linear_velocity
                desired_angular_velocity = self.prev_angular_velocity + delta_angular_velocity

                # Update previous velocities
                self.prev_linear_velocity = desired_linear_velocity
                self.prev_angular_velocity = desired_angular_velocity

                # Position control
                msg.linear.x = desired_linear_velocity

                # Orientation control
                goal_theta = desired_angular_velocity
                
                diff = goal_theta - self.pose_.theta
                if diff > math.pi:
                    diff -= 2 * math.pi
                elif diff < -math.pi:
                    diff += 2 * math.pi

                msg.angular.z = 6 * diff

                data = Movement()
                data.velocity = desired_linear_velocity
                data.acceleration = delta_linear_velocity
                self.plot_movement.publish(data)

            else:
                msg.linear.x = 0.0
                msg.angular.z = math.pi / 2

                if not self.flag_moveup:
                    if self.target_x == 1.0:
                        self.target_x = 10.0
                    else:
                        self.target_x = 1.0
                    
                    self.flag_moveup = True

                elif self.flag_moveup:
                    self.target_y += 1.0
                    self.flag_moveup = False
                
                self.desired_position = True
                self.desired_orientation = False

            self.cmd_vel_publisher_.publish(msg)

        elif not self.desired_orientation:
            if self.flag_moveup:
                if self.target_x == 10.0:
                    self.move_desired_yaw(0.0)
                else:
                    self.move_desired_yaw(3.14)

            elif not self.flag_moveup:
                self.move_desired_yaw(1.57)


    
    def limit_velocity_change(self, delta_velocity):
        # Limit velocity change to ensure it does not exceed acceleration or deceleration limits
        if delta_velocity > self.max_acceleration:
            return self.max_acceleration
        elif delta_velocity < self.max_deceleration:
            return self.max_deceleration
        else:
            return delta_velocity

    def move_desired_yaw(self,desired_yaw):

        error = desired_yaw - self.pose_.theta
        if abs(error) > 0.01:
            self.integral += error
            derivative = error - self.prev_error_orie
            angular_velocity = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = angular_velocity
            self.cmd_vel_publisher_.publish(twist)
            self.prev_error_orie = error
        else:
            self.desired_position = False
            self.desired_orientation = True

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
