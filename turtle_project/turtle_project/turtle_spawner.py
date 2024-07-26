#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import MoveTurtle
from my_custom_interfaces.msg import Coordinates2D
from turtlesim.srv import Spawn
from turtlesim.srv import Kill

import random 

class spawnerNode(Node):
    def __init__(self):
        super().__init__("Spawner_node")
        self.client = self.create_client(MoveTurtle, "Move")
        self.__spawner_client = self.create_client(Spawn, "spawn")
        self.__killer_client = self.create_client(Kill, "kill")

        self.__res = bool()

        self.turtles_coordinates = []
        self.turtles_names = []

        # the timer responsible for spawning the turtles
        self.__turtle_spawner_timer = self.create_timer(random.uniform(1.8, 3.0), self.__spawner_service_cb)

        # the main timer function that calls the move service
        self.__move_call_timer = self.create_timer(0.5, self.__main_cb)

    def __killer_service_call(self, turtle_name):

        while not self.__killer_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the server to be available...")
        
        msg = Kill.Request()
        msg.name = turtle_name

        self.__killer_future = self.__killer_client.call_async(msg)

        self.__killer_future.add_done_callback(self.__killer_future_done_cb)

    def __killer_future_done_cb(self, future):
        try:
            self.turtles_coordinates.pop(0)
            self.turtles_names.pop(0)
        except:
            self.get_logger().error("response failed.... :(")


    def __spawner_service_cb(self):

        while not self.__spawner_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the server to be available...")

        r_x = random.uniform(0.0, 11.0)
        r_y = random.uniform(0.0, 11.0)
        
        msg = Spawn.Request()
        msg.x = r_x
        msg.y = r_y

        self.curr_c = Coordinates2D()
        self.curr_c.x = r_x
        self.curr_c.y = r_y

        self.__spawn_future = self.__spawner_client.call_async(msg)

        self.__spawn_future.add_done_callback(self.__spawner_future_done_cb)
        
        print(self.turtles_names)

    def __spawner_future_done_cb(self, future):
        try:
            name = future.result().name
            self.get_logger().info(f"New turtle: {name} has been spawned")
            self.turtles_coordinates.append(self.curr_c)
            self.turtles_names.append(name)

        except:
            self.get_logger().error("response failed.... :(")

    def __main_cb(self):
        if len(self.turtles_coordinates) == 0:
            pass
        elif self.call_Move_service(coordinates=self.turtles_coordinates[0]):
            #self.turtles_coordinates.pop(0)
            #self.turtles_names.pop(0)
            self.__killer_service_call(turtle_name=self.turtles_names[0])

    def call_Move_service(self, coordinates):

        while not self.client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the server to be available...")

        req = MoveTurtle.Request()
        req.coordinates = coordinates

        self.future = self.client.call_async(request=req)

        self.future.add_done_callback(self.__cb_future_done)

        print(self.__res)

        return self.__res

    def __cb_future_done(self, future):
        try:
            self.__res = future.result().success
            self.get_logger().info(f"the response is: {self.__res}")
            if self.__res == True:
                print("the request has been processed successfully")
            else:
                print("the turtle is busy now ...")
        except:
            self.get_logger().error("response failed.... :(")

def main(args=None):
    rclpy.init(args=args) # to start ros2 communication

    node = spawnerNode()

    rclpy.spin(node)
    

    rclpy.shutdown() # to end ros2 communication

