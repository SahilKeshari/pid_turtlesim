import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TurtlePose
import math
import random
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty

class TurtleRotate(Node):
    def __init__(self):
        super().__init__('turtle_rotate')
        self.pose_ = None
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber_ = self.create_subscription(TurtlePose,'/turtle1/pose',self.callback_pose,10)

        self.normal_pose_publisher_ = self.create_publisher(TurtlePose, '/rt_real_pose', 10)
        self.noisy_pose_publisher_ = self.create_publisher(TurtlePose,'/rt_noisy_pose',10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.noise_timer = self.create_timer(5.0,self.add_gaussian_noise)
        self.publish_topic_timer = self.create_timer(5.0,self.publish_topics)

        self.call_relocate_server(5.25,1.0,0.0)

        self.noise_pos_x = 0.0
        self.noise_pos_y = 0.0
        self.noise_orie = 0.0
        
        self.speed = 2.0
        self.radius = 5.0

    def callback_pose(self,msg):
        self.pose_ = msg

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

    def control_loop(self):
        msg = Twist()
        msg.linear.x = self.speed 
        self.angular_velocity = self.speed / self.radius
        msg.angular.z = self.angular_velocity
        # msg.angular.z = self.angular_velocity + self.noise    # to add the gaussian noise 

        self.publisher_.publish(msg)

    def add_gaussian_noise(self):
        self.noise_pos_x = random.gauss(0, 1.0)
        self.noise_pos_y = random.gauss(0, 1.0)
        self.noise_orie = random.gauss(0,0.3)

    def publish_topics(self):
        if self.pose_ == None:
            return
        
        normal_pose = TurtlePose()
        normal_pose.x = self.pose_.x
        normal_pose.y = self.pose_.y
        normal_pose.theta = self.pose_.theta
        self.normal_pose_publisher_.publish(normal_pose)

        noisy_pose = TurtlePose()
        noisy_pose.x = self.pose_.x + self.noise_pos_x
        noisy_pose.y = self.pose_.y + self.noise_pos_y
        noisy_pose.theta = self.pose_.theta + self.noise_orie
        self.noisy_pose_publisher_.publish(noisy_pose)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleRotate()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
