#1/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(String, 'map_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_map)

    def publish_map(self):
        # Create a 2D map representing connections between nodes
        # For example:
        map_data = """
        A B 5
        B C 3
        C D 2
        A D 7
        """
        self.publisher_.publish(String(data=map_data))
        self.get_logger().info('Published map data: "%s"' % map_data)

def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()