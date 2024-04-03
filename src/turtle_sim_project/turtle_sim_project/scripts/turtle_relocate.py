#! /usr/bin/env python3
import rclpy
from rclpy.node import Node 
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
import random
import math

class Turtle_relocate(Node):
    
    def __init__(self):
        super().__init__("turtle_relocate")
        self.spawn_turtle_timer_ = self.create_timer(5.0,self.relocate_turtle)

    def relocate_turtle(self):
        x = random.uniform(0.0,10.0)
        y = random.uniform(0.0,10.0)
        theta = random.uniform(0.0,2*math.pi)
        self.call_relocate_server(x,y,theta)

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
    

def main(args=None):
    rclpy.init(args=args)
    node = Turtle_relocate()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
