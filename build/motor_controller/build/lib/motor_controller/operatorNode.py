#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class controller(Node):
    def __init__(self):
        super().__init__("controller")

        # Subscribe to twist no and create publisher
        self.twist_sub_ = self.create_subscription(Twist, "cmd_vel", self.incoming)
        self.twist_pub_ = self.create_publisher(Twist, "controller_input", 10)
        self.get_logger().info("Started Listener")

        # Define incoming function

    def incoming(self, msg: Twist):
        self.get_logger().info("Message received")
        self.twist_pub_.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()