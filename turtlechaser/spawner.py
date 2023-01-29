#!/usr/bin/env python3

import random
from functools import partial

import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose

class Spawner(Node):
    
    def __init__(self):
        super().__init__("spawner")
        self.x_ = 0
        self.y_ = 0
        self.name_ = ""
        self.spawn_turtle()
        
        self.chaser_position_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.callback_chased, 10)
        self.timer_ = self.create_timer(10.0, self.spawn_turtle)
        
    def callback_chased(self, chaser_position):
        if chaser_position.x - self.spawned_x_ < 0.4 and chaser_position.y - self.spawned_y_ < 0.4:
            self.kill_turtle()
        else: pass
    
    def kill_turtle(self):
        turtle_killer = self.create_client(Kill, "kill")
        
        turtle_to_kill = Kill.Request()
        turtle_to_kill.name = self.name_
        
        future = turtle_killer.call_async(turtle_to_kill)
        future.add_done_callback(self.callback_killed_turtle)
        
    def callback_killed_turtle(self, future):
        try:
            self.get_logger().info("Chased the turtle!")
        except Exception as e:
            self.get_logger().error(e)
    
    def spawn_turtle(self):
        spawner = self.create_client(Spawn, "spawn")
        while not spawner.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the chaser to spawn..")
            
        turtle_to_spawn = Spawn.Request()
        turtle_to_spawn.x = round(random.uniform(0.0, 11.0), 3)
        turtle_to_spawn.y = round(random.uniform(0.0, 11.0), 3)
        turtle_to_spawn.theta = random.uniform(-3, 3)
        turtle_to_spawn.name = "chased"
        
        self.x_ = turtle_to_spawn.x
        self.y_ = turtle_to_spawn.y
        self.name_ = turtle_to_spawn.name
        
        future = spawner.call_async(turtle_to_spawn)
        future.add_done_callback(partial(self.future_callback, spawned_x=turtle_to_spawn.x, spawned_y=turtle_to_spawn.y))
        
    def future_callback(self, future, spawned_x, spawned_y):
        try:
            response = future.result()
            self.get_logger().info(f"Spawned {response.name} ==> x: {spawned_x}, y: {spawned_y}")
        except Exception as e:
            self.get_logger().error(e);               

def main(args=None):
    rclpy.init(args=args)
    spawner_node = Spawner()
    rclpy.spin(spawner_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
