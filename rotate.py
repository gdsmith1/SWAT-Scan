import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.vel_msg = Twist()
        
    def rotate_45_degrees(self):
        angular_speed = 0.7854
        rotation_duration = 1

        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = angular_speed

        self.publisher.publish(self.vel_msg)
        self.get_logger().info("Rotating 45 degrees")

        time.sleep(rotation_duration)

        self.vel_msg.angular.z = 0.0
        self.publisher.publish(self.vel_msg)
        self.get_logger().info("Stopping rotation")

    def full_rotation(self):
        for _ in range(8):
            self.rotate_45_degrees()
            time.sleep(4)

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_controller = TurtleBot3Controller()

    try:
        turtlebot3_controller.full_rotation()
    except KeyboardInterrupt:
        pass
    finally:
        turtlebot3_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()