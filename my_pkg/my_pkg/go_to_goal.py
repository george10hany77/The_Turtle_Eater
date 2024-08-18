#!/usr/bin/env python3
import imp
from tokenize import String
from sympy import false, im, true
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from example_interfaces.msg import String
import math
import sys
from my_custom_interfaces.srv import MoveTurtle
from my_custom_interfaces.msg import Coordinates2D
from turtlesim.srv import Kill


class goToNode(Node):

    def __init__(self):
        super().__init__("Go_To_node")
        self.myPub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.mySub = self.create_subscription(Pose, "/turtle1/pose", self.update_curr_pose, 10)
        self.move_server = self.create_service(MoveTurtle, "moveTurtle", self.cb_move_turtle)
        self.kill_client = self.create_client(Kill, "/kill")
        self.name_sub = self.create_subscription(String, "/names", self.cb_names, 10)
        self.create_timer(0.1, self.cb_go)
        self.get_logger().info("The GO_TO node has been started....")
        self.current_pose = Pose()    
        self.points:Coordinates2D = []
        self.names:String = [] 

    def kill_turtle(self, name:String):

        while not self.kill_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the kill server....")

        req = Kill.Request()
        req.name = name

        # I will not handle the future object
        self.kill_client.call_async(req)

    def cb_names(self, msg:String):
        self.names.append(msg.data)

    def cb_move_turtle(self, request, response):
        self.points.append(request.coordinates)
        response.success = True
        return response

    def update_curr_pose(self, current_position:Pose):
        self.current_pose = current_position
        #self.get_logger().info(f"position: x: {self.current_pose.x}, y: {self.current_pose.y}, theta: {self.current_pose.theta}")

    def cb_go(self):
        if len(self.points) != 0:
            x = self.points[0].x
            y = self.points[0].y

            if self.go_to_x_y(x, y):
                # case the turtle has reached the target
                #self.kill_turtle(name=self.names[0])
                self.points.pop(0)
                #self.names.pop(0)
        else:
            self.get_logger().info("The turtle is dead :(")

    def go_to_x_y(self, x, y):
        target_x = x
        target_y = y

        angular_tolerance = 0.0001
        linear_tolerance = 0.1

        msg = Twist()

        # angle calculation
        kpa = 3.5
        goal_theta = math.atan2(target_y - self.current_pose.y, target_x - self.current_pose.x)
        angular_error = (goal_theta - self.current_pose.theta) * kpa

        # distance calculation
        kpl = 2.8
        distance = math.sqrt((target_y - self.current_pose.y)**2 + (target_x - self.current_pose.x)**2)
        linear_error = distance * kpl
        
        if math.fabs(angular_error) < angular_tolerance:
            angular_error = 0.0
            if math.fabs(linear_error) > linear_tolerance:
                msg.linear.x = linear_error
            else:
                msg.linear.x = 0.0        

        msg.angular.z = angular_error

        # publishing the data
        self.myPub.publish(msg=msg)

        if math.fabs(angular_error) < angular_tolerance and math.fabs(linear_error) < linear_tolerance:     
            self.get_logger().info(f"The Goal x: {self.current_pose.x}, y: {self.current_pose.y} has been reached")
            #self.get_logger().info(f"The Goal has been reached")
            return true
        else:
            self.get_logger().info(f"Distance: {linear_error}, Alpha: {angular_error}")
            return false


    def go_to(self):
        target_x = float(sys.argv[1])
        target_y = float(sys.argv[2])  

        angular_tolerance = 0.05
        linear_tolerance = 0.1

        msg = Twist()

        # angle calculation
        kpa = 1.1
        goal_theta = math.atan2(target_y - self.current_pose.y, target_x - self.current_pose.x)
        angular_error = (goal_theta - self.current_pose.theta) * kpa

        # distance calculation
        kpl = 1.1
        distance = math.sqrt((target_y - self.current_pose.y)**2 + (target_x - self.current_pose.x)**2)
        linear_error = distance * kpl
        
        if math.fabs(angular_error) < angular_tolerance:
            angular_error = 0.0
            if math.fabs(linear_error) > linear_tolerance:
                msg.linear.x = linear_error
            else:
                msg.linear.x = 0.0        

        msg.angular.z = angular_error

        if math.fabs(angular_error) < angular_tolerance and math.fabs(linear_error) < linear_tolerance:     
            self.get_logger().info(f"The Goal x: {self.current_pose.x}, y: {self.current_pose.y} has been reached")
            #self.get_logger().info(f"The Goal (x: {target_x}, y: {target_y}) has been reached")
        else:
            self.get_logger().info(f"Distance: {linear_error}, Alpha: {angular_error}")

        self.myPub.publish(msg=msg)

def main(args=None):
    rclpy.init(args=args) # to start ros2 communication

    node = goToNode()
    rclpy.spin(node)

    rclpy.shutdown() # to end ros2 communication

