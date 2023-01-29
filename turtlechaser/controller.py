#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleController(Node):
    
    def __init__(self):
        super().__init__("controller")
        self.chaser_x_ = 0
        self.chaser_y_ = 0
        
        self.position_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.set_position, 10)        
        self.velocity_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        
    def set_position(self, msg):
        self.chaser_x_ = msg.x
        self.chaser_y_ = msg.y
        
        self.chaser_to_turtle()
        
    def chaser_to_turtle(self):
        velocity = Twist()
        
        if(self.chaser_x_ < 8.0):
            velocity.linear.x = 2.0
        else:
            velocity.linear.x = 0.0
        
        self.velocity_publisher_.publish(velocity)

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
