import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class ZigzagTurtle(Node):
    def __init__(self):
        super().__init__('zigzag_turtle')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.current_pose = None
        self.left_corner = (0.5, 0.5)  # Define left corner position
        self.right_corner = (9.5, 0.5)  # Define right corner position
        self.target_position = None
        self.move_up_after_corner = True

    def pose_callback(self, msg):
        self.current_pose = msg

    def move_to_next_position(self):
        if self.current_pose is not None:
            # Check if the turtle needs to move up after reaching a corner
            if self.move_up_after_corner:
                msg = Twist()
                msg.linear.x = 2  # Move up
                self.publisher.publish(msg)
                self.move_up_after_corner = False
                return

            # Calculate the target position based on the current pose
            if self.target_position is None or self.target_position == self.right_corner:
                self.target_position = self.left_corner
            else:
                self.target_position = self.right_corner

            # Calculate distance and angle to the target position
            dist_x = self.target_position[0] - self.current_pose.x
            dist_y = self.target_position[1] - self.current_pose.y
            distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
            angle_to_target = math.atan2(dist_y, dist_x)

            # Calculate angle difference
            angle_diff = angle_to_target - self.current_pose.theta
            # Adjust angle difference to range [-pi, pi]
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            msg = Twist()
            # If the turtle hasn't reached the desired angle yet, rotate
            if abs(angle_diff) > 0.1:
                msg.angular.z = min(max(2 * angle_diff, -2), 2)  # Limit angular velocity to [-2, 2]
            else:
                # If the turtle is close to the desired angle, move forward
                msg.linear.x = min(max(2 * distance, -2), 2)  # Limit linear velocity to [-2, 2]
                # Reset angular velocity to zero for straight line movement
                msg.angular.z = 0
                # Reset target position
                self.target_position = None
                # Indicate that the turtle should move up after reaching the corner
                self.move_up_after_corner = True

            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZigzagTurtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
