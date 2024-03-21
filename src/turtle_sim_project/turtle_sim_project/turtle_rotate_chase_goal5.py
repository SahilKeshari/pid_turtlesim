import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TurtlePose
from turtlesim.srv import Spawn
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
from custom_msgs.msg import Movement

import math
import random

class TurtleRotate(Node):
    def __init__(self):
        super().__init__('turtle_rotate5')
        self.rt_pose_ = None
        self.rt_publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.rt_pose_subscriber_ = self.create_subscription(TurtlePose,'/turtle1/pose',self.rt_callback_pose,10)

        self.rt_pose_publisher_ = self.create_publisher(TurtlePose,'rt_real_pose',10)

        self.pt_pose_ = None
        self.pt_publisher_ = self.create_publisher(Twist, '/police_turtle/cmd_vel', 10)
        self.pt_pose_subscriber_ = self.create_subscription(TurtlePose,'/police_turtle/pose',self.pt_callback_pose,10)

        self.call_relocate_server(5.25,1.0,0.0)

        self.rt_timer = self.create_timer(0.1, self.rt_rotate_loop)

        self.pt_timer = self.create_timer(10.0, self.pt_start_callback)

        self.speed = 3.0
        self.radius = 5.0

        self.prev_linear_velocity = 0.0
        self.prev_angular_velocity = 0.0

        # maximum acceleration and deceleration limits
        self.max_acceleration = 2.0 
        self.max_deceleration = -2.0

        self.dist_x = None
        self.dist_y = None

        self.pt_plot_movement = self.create_publisher(Movement,'pt/movement_data',10)


    def call_relocate_server(self,x,y,theta):
        client_clear = self.create_client(Empty,'clear')
        while not client_clear.wait_for_service(1.0):
            self.get_logger().warn('waiting for server...')
            
        clear_request = Empty.Request()
        future = client_clear.call_async(clear_request)

        client = self.create_client(TeleportAbsolute,'turtle1/teleport_absolute')
        while not client.wait_for_service(1.0):
            self.get_logger().warn('waiting for server...')
        
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta

        client.call_async(request)

        clear_request = Empty.Request() #reclear to remove the track marks
        future = client_clear.call_async(clear_request)

    def call_spawn_server(self,turtle_name,x,y,theta):
        client = self.create_client(Spawn,'spawn')
        while not client.wait_for_service(1.0):
            self.get_logger().warn('waiting for server...')
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client.call_async(request)

        # future.add_done_callback(self.callback_call_spawn) callback function used for checking

    def pt_start_callback(self):
        x = random.uniform(0.0,10.0)
        y = random.uniform(0.0,10.0)
        theta = random.uniform(0.0,2*math.pi)

        # x,y,theta = 1.0,1.0,0.0 # for testing purpose

        self.call_spawn_server('police_turtle',x,y,theta)
        self.chase_timer = self.create_timer(0.1,self.pt_chase_loop)
        self.get_dist_timer = self.create_timer(5.0,self.get_dist)
        self.dist_subscriber = self.create_subscription(TurtlePose,'/rt_real_pose',self.get_dist,10)
        self.check_dist_timer = self.create_timer(0.01,self.check_dist)

        self.pt_timer.cancel()

    def rt_callback_pose(self,msg):
        self.rt_pose_ = msg

        rt_pose = TurtlePose()
        rt_pose.x = self.rt_pose_.x
        rt_pose.y = self.rt_pose_.y
        rt_pose.theta = self.rt_pose_.theta

        self.rt_pose_publisher_.publish(rt_pose)

    def pt_callback_pose(self,msg):
        self.pt_pose_ = msg

    def rt_rotate_loop(self):
        msg = Twist()
        msg.linear.x = self.speed 
        self.angular_velocity = self.speed / self.radius
        msg.angular.z = self.angular_velocity
        self.rt_publisher_.publish(msg)
    
    def pt_chase_loop(self):
        if self.rt_pose_ == None or self.pt_pose_ == None:
            return

        # self.get_logger().info(f' {self.dist_x} {self.dist_y}')
        if self.dist_x == None or self.dist_y == None:
            return


        initial_angle_rad = math.atan2(self.dist_y, self.dist_x)
        initial_angle_deg = math.degrees(initial_angle_rad)
        new_angle_deg = initial_angle_deg + (12 * self.speed * 5)
        new_angle_rad = math.radians(new_angle_deg)

        # to estimate the location of pt after 5 secs
        estimate_dist_x = self.radius * math.cos(new_angle_rad)
        estimate_dist_y = self.radius * math.sin(new_angle_rad)

        msg = Twist()
        distance = (math.sqrt(estimate_dist_x * estimate_dist_x + estimate_dist_y * estimate_dist_y))

        if distance > 3.0:
            desired_linear_velocity = 2*distance
            desired_angular_velocity = math.atan2(self.dist_y, self.dist_x)

            # Calculate velocity changes
            delta_linear_velocity = desired_linear_velocity - self.prev_linear_velocity
            delta_angular_velocity = desired_angular_velocity - self.prev_angular_velocity

            delta_linear_velocity = self.limit_velocity_change(delta_linear_velocity)
            delta_angular_velocity = self.limit_velocity_change(delta_angular_velocity)
            
            # Apply acceleration and deceleration limits
            desired_linear_velocity = self.prev_linear_velocity + delta_linear_velocity
            desired_angular_velocity = self.prev_angular_velocity + delta_angular_velocity

            diff = desired_angular_velocity - self.pt_pose_.theta

            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi

            desired_linear_velocity = min(desired_linear_velocity,self.speed/2)    
            msg.linear.x = desired_linear_velocity
            msg.angular.z = 6*diff 

            self.prev_linear_velocity = desired_linear_velocity
            self.prev_angular_velocity = desired_angular_velocity

            data = Movement()
            data.velocity = desired_linear_velocity
            data.acceleration = delta_linear_velocity
            self.pt_plot_movement.publish(data)

        else:
            self.speed = 0.0

        self.pt_publisher_.publish(msg)

    def limit_velocity_change(self, delta_velocity):
        # Limit velocity change to ensure it does not exceed acceleration or deceleration limits
        if delta_velocity > self.max_acceleration:
            return self.max_acceleration
        elif delta_velocity < self.max_deceleration:
            return self.max_deceleration
        else:
            return delta_velocity

    def get_dist(self,msg=None):
        if self.pt_pose_ == None or msg == None:
            return
        
        self.dist_x = msg.x - self.pt_pose_.x
        self.dist_y = msg.y - self.pt_pose_.y
    
    def check_dist(self):
        if self.rt_pose_ == None or self.pt_pose_ == None:
            return
        
        dist_x = self.rt_pose_.x - self.pt_pose_.x
        dist_y = self.rt_pose_.y - self.pt_pose_.y
        distance = (math.sqrt(dist_x * dist_x + dist_y * dist_y))

        if distance < 3.0:
            self.speed = 0.0
            self.get_logger().info('The chase is complete!!!')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleRotate()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
