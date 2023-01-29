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
        self.spawned_x_ = 0
        self.spawned_y_ = 0
        
        self.spawned_position_subscriber_ = self.create_subscription(Pose, "spawned/pose", self.set_spawned_position, 10)        
        self.chaser_position_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.set_chaser_position, 10)        
        self.velocity_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        
    def set_spawned_position(self, position):
        self.spawned_x_ = position.x
        self.spawned_y_ = position.y
        
    def set_chaser_position(self, position):
        self.chaser_x_ = position.x
        self.chaser_y_ = position.y
        
        self.chaser_to_turtle()
        
    def chaser_to_turtle(self):
        velocity = Twist()
        
        if(self.chaser_x_ < self.spawned_x_):
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
