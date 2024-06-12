#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("first_node")
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello"+str(self.counter_))
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args)
    # Create a node
    node = MyNode()

    #run some stuff before that run spin to keep the node alive to communicate with it
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
