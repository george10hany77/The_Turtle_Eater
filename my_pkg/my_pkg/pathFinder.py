#!/usr/bin/env python3
import sys
from sympy import im
from my_custom_interfaces.msg import Coordinates2D
import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import MoveTurtle
from .AStarNode import AStarNode
from .PathFinding import PathFinding
from turtlesim.srv import Spawn
from example_interfaces.msg import String
from asyncio import Future
from turtlesim.msg import Pose

class PFNode(Node):

    def __init__(self):
        super().__init__("Path_finder")
        self.turtle_client = self.create_client(MoveTurtle, "moveTurtle")
        self.turtle_spawner_client = self.create_client(Spawn, "/spawn")
        self.curr_pose = None
        # initializing the map
        ob1 = AStarNode(6,7)
        ob2 = AStarNode(4,3)
        ob3 = AStarNode(4,4)
        ob4 = AStarNode(3,4)
        ob5 = AStarNode(2,0)
        ob6 = AStarNode(2,1)
        ob7 = AStarNode(2,2)
        ob8 = AStarNode(2,3)
        ob9 = AStarNode(2,3)
        ob10 = AStarNode(3,5)
        ob11 = AStarNode(3,6)
        ob12 = AStarNode(2,3)
        self.obstacles = [ob1, ob2, ob3, ob4, ob5, ob6, ob7, ob8, ob9, ob10, ob11, ob12] 
        # (11, 11) is the size of the grid
        self.path_finder = PathFinding(11,11)
        self.path_finder.mapList(self.obstacles)
        print("the list is mapped")
        self.path_finder.add_obstacles(self.obstacles)
        self.obstacles_2 = self.path_finder.mapList_2(self.obstacles)
        self.path:list[AStarNode] = []
        self.flag = True
        self.start_moving_to_goal = False
        self.no_more_spawning = False
        self.spawn_timer = self.create_timer(1.0, self.spawn_obstacles)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.cb_pose_sub, 20)
        self.turtle_mover = self.create_timer(0.1, self.make_move)

    def make_move(self):
        if self.start_moving_to_goal:
            if self.path != None:
                if len(self.path) > 0:
                    while not self.turtle_client.wait_for_service(1.0):
                        self.get_logger().warn("Waiting for the move server....")

                    r_x = int(self.path[0].get_x())
                    r_y = int(self.path[0].get_y())
                    coordinates = Coordinates2D()
                    coordinates.x = float(r_x)
                    coordinates.y = float(r_y)
                    req = MoveTurtle.Request()
                    req.coordinates = coordinates
                    future = self.turtle_client.call_async(req)
                    future.add_done_callback(self.done_cb)
                else:
                    print("the path has been sent")
                    self.turtle_mover.destroy()
                
            else:
                print("the path is None")

    def done_cb(self, future:Future):
        try:
            res = future.result()
            self.get_logger().info(f"The result: {res.success}")
            self.path.pop(0)
        except:
            self.get_logger().error("error from done_cb!!")


    def cb_pose_sub(self, msg: Pose):
        if self.curr_pose == None: # to get the position only once
            self.curr_pose = msg
            print(self.curr_pose)
        elif self.flag:
            self.flag = False
            sx = int(self.curr_pose.x)
            sy = int(self.curr_pose.y)
            start = AStarNode(sx, sy)
            end = AStarNode(0,0)
            self.path_finder.map(end)
            self.path_finder.map(start)
            self.path = self.path_finder.a_star(start=start, end=end)
            print("the algo has been ran")
            print(f"the path: {self.path}")

    def spawn_obstacles(self):
        if len(self.obstacles_2) > 0:
            self.spawn_turtle(self.obstacles_2[0])
        else:
            print("no more obstacles")
            # start moving
            self.start_moving_to_goal = True
            self.spawn_timer.destroy()
            self.no_more_spawning = True

    def spawn_turtle(self, node):
        if not self.no_more_spawning:
            while not self.turtle_spawner_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for the turtle spawner server.....")
            req = Spawn.Request()
            req.x = float(node.x)
            req.y = float(node.y)
            future = self.turtle_spawner_client.call_async(request=req)
            future.add_done_callback(self.cb_spawner)
    
    def cb_spawner(self, future:Future):
        try:
            res = future.result()
            self.get_logger().info(f"The spawned turtle's name: {res.name}")
            print(self.obstacles_2)
            self.obstacles_2.pop(0)
        except:
            self.get_logger().error("error from cb_spawner!!")

def main(args=None):
    rclpy.init(args=args)

    node = PFNode()
    rclpy.spin(node)

    rclpy.shutdown()
