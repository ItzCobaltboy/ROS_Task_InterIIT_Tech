#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        
        # Subscribe to teleop keyboard output and publish to diff_drive_controller
        self.twist_sub_ = self.create_subscription(
            Twist, 
            "/cmd_vel", 
            self.incoming, 
            10
        )
        self.twist_pub_ = self.create_publisher(Twist, "/diff_drive_controller/cmd_vel_unstamped", 10 )# Changed to correct topic
        self.get_logger().info("Started Controller Node")

    def incoming(self, msg: Twist):
        self.get_logger().info(f"Received velocity - linear: {msg.linear.x}, angular: {msg.angular.z}")
        self.twist_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()