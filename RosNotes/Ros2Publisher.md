# Ros Publisher 
A ros publisher is a node that sends the information it has / produces into a topic . This way it can forward the information to the necessary nodes that are subscribed to that topic .

- go to the package folder with scripts 
- create a new file called "draw_circle.py"
- use the following code:

```
#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
# import geometry messages 
from geometery_msgs.msg import Twist # don't forget to add the geometry_msgs tag as depends in the package.xml file as well as turtlesim 

def DrawCircleNode(Node):
    
    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_ = self.create.publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(0.5,self.send_velocity_command)
        self.get_logger().info("Draw Circle node has been started") 

    def send_velocity_command(self):
        msg = Twist()
        # create a message
        msg.linear.x = 2.0
        msg.angular.z = 1.0

        # publish the message 
        self.cmd_vel_pub_.publish(msg)

def main():
    rclpy.init(args = args)
    node = DrawCircleNode()
    rclpy.spin(node) # keep the node alive 
    rclpy.shutdown()

```

Note:         self.cmd_vel_pub_ = self.create.publisher(Message type,topic name, value | callback group | event callbacks | etc.)

Note : to add the node into the command -> setup.py -> draw_circle = my_robot_controller.draw_circle:main

Note: colcon build --symlink-install -> only work for exec that have already created and it keeps it live ,i.e every call shall re compile the package 


that's it ! the first publisher is written !