#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')
        
        # Publisher to /cmd_vel topic
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Define the Twist message
        self.vel_msg = Twist()
        
    def move_one_inch(self):
        # Set linear speed in meters per second
        linear_speed = 0.02  # meters per second (small speed for better precision)
        distance_to_move = 0.0254  # 1 inch in meters
        time_to_move = distance_to_move / linear_speed  # time = distance / speed

        # Set the forward linear speed
        self.vel_msg.linear.x = linear_speed
        self.vel_msg.angular.z = 0.0  # No rotation

        # Start moving forward
        self.publisher.publish(self.vel_msg)
        self.get_logger().info(f"Moving forward 1 inch ({distance_to_move} meters)")

        # Wait for the calculated time to achieve 1 inch distance
        time.sleep(time_to_move)

        # Stop movement
        self.vel_msg.linear.x = 0.0
        self.publisher.publish(self.vel_msg)
        self.get_logger().info("Stopping")

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_controller = TurtleBot3Controller()

    try:
        # Execute the move_one_inch command
        turtlebot3_controller.move_one_inch()
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node and shutdown
        turtlebot3_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
