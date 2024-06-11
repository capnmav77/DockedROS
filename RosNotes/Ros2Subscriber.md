# Ros2 Subscriber
it is a node that recieves the data froma topic by subscribing to the topic 

- for this let's focus in turtlesim -> /msg/pose 


```
#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscribeNode(Node):

    def __init__(self):
        super().__init__("pose subscriber")
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

    def pose_callback(self,msg: Pose):
        self.get_logger().info(str(msg))
    

def main():
    rclpy.init(args=args)
    Node = PoseSubscribeNode()
    rclpy.spin(Node)
    rclpy.shutdown()


```
Note : self.pose_subscriber_ = self.create_subscription(message name, topic name , callback func , queue size = 10 / buffer)

we can also individually call the messages : msg.x , msg.y etc. 