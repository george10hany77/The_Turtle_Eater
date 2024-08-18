#!/usr/bin/env python3
from asyncio import Future
import re
from my_custom_interfaces.msg import Coordinates2D
import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import MoveTurtle
from turtlesim.srv import Spawn
import random
from example_interfaces.msg import String


class trajectoryNode(Node):
    def __init__(self):
        super().__init__("My_service_node")
        self.client = self.create_client(MoveTurtle, "moveTurtle")
        self.spawnerClient = self.create_client(Spawn, "/spawn")
        self.names_publisher = self.create_publisher(String, "/names", 10)
        self.timer = self.create_timer(random.uniform(3, 4.5), self.make_move)

    def spawn_turtle(self, x, y):
        
        while not self.spawnerClient.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the spawn server....")   

        sp = Spawn.Request()
        sp.x = x
        sp.y = y

        future = self.spawnerClient.call_async(sp) 

        future.add_done_callback(self.cb_spawner)

    def cb_spawner(self, future:Future):
        try:
            res = future.result()
            self.get_logger().info(f"The spawned turtle's name: {res.name}")
            msg = String()
            msg.data = res.name
            self.names_publisher.publish(msg=msg)
        except:
            self.get_logger().error("error !!")

    def make_move(self):
        
        while not self.client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the move server....")

        r_x = round(random.uniform(0.0, 11.0), 0) # making the map smaller
        r_y = round(random.uniform(0.0, 11.0), 0) # making the map smaller
        
        self.spawn_turtle(r_x, r_y)

        coordinates = Coordinates2D()
        coordinates.x = r_x
        coordinates.y = r_y

        req = MoveTurtle.Request()
        req.coordinates = coordinates

        future = self.client.call_async(req)

        future.add_done_callback(self.done_cb)

    def done_cb(self, future:Future):
        try:
            res = future.result()
            self.get_logger().info(f"The result: {res.success}")
        except:
            self.get_logger().error("error !!")


def main(args=None):
    rclpy.init(args=args)
    node = trajectoryNode()
    rclpy.spin(node)
    rclpy.shutdown()
    