# Ros2 Closed Loop Controller 
they are controllers that form a closed loop of publishing and subscribing 

- we subscribe to a topic and then publish to a topic which is used by others 

```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from turtlesim.msg import Pose

def TurtleControllerNode(Node):
    
    def __init__(self):
        super().__init__('turtle_controller')
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel" , 10)
        self.pose_subscriber_ = self.create_subscription(Pose,"/turtle1/pose",self.pose_callback, 10)

        self.get_logger().info("Turtle controller has been started")

    def pose_callback(self, Pose):
        cmd = Twist()

        if pose.x > 9.0 or pose.x<2 or pose.y>9.0 or pose.y<2.0 >:
            cmd.linar.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
            self.cmd_vel_publisher_.publish(cmd)


def main():
    rclpy.init()
    node = Node('TurtleControllerNode')
    rclpy.spin(node)
    rclpy.shutdown()
```


here : 
    turtle1/cmd_val -> turtle sim -> turtle1/pose -> turtle controller -> turtle1/cmd_val