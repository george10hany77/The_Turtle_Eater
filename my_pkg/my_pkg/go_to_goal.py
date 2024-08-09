#!/usr/bin/env python3
import imp
from sympy import false, im, true
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import sys
from my_custom_interfaces.msg import Coordinates2D

class goToNode(Node):

    def __init__(self):
        super().__init__("Go_To_node")
        self.myPub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.mySub = self.create_subscription(Pose, "/turtle1/pose", self.update_curr_pose, 10)
        self.create_timer(0.1, self.cb_go)
        self.get_logger().info("The GO_TO node has been started....")
        self.current_pose = Pose()
                
        c1 = Coordinates2D()
        c1.x = 3.0
        c1.y = 4.0

        c2 = Coordinates2D()
        c2.x = 2.0
        c2.y = 2.0

        c3 = Coordinates2D()
        c3.x = 1.0
        c3.y = 9.0

        c4 = Coordinates2D()
        c4.x = 11.0
        c4.y = 8.0

        c5 = Coordinates2D()
        c5.x = 2.0
        c5.y = 10.0

        c6 = Coordinates2D()
        c6.x = 11.0
        c6.y = 11.0

        c7 = Coordinates2D()
        c7.x = 0.0
        c7.y = 0.0

        self.points = [c1, c2, c3, c4, c5, c6, c7]

    def update_curr_pose(self, current_position:Pose):
        self.current_pose = current_position
        #self.get_logger().info(f"position: x: {self.current_pose.x}, y: {self.current_pose.y}, theta: {self.current_pose.theta}")

    def cb_go(self):
        if len(self.points) != 0 and self.go_to_x_y(self.points[0].x, self.points[0].y):
            self.points.pop(0)
        elif len(self.points) == 0:
            self.get_logger().info("The turtle is dead :(")

    def go_to_x_y(self, x, y):
        target_x = x
        target_y = y

        angular_tolerance = 0.05
        linear_tolerance = 0.1

        msg = Twist()

        # angle calculation
        kpa = 3.8
        goal_theta = math.atan2(target_y - self.current_pose.y, target_x - self.current_pose.x)
        angular_error = (goal_theta - self.current_pose.theta) * kpa

        # distance calculation
        kpl = 2.2
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
            #self.get_logger().info(f"The Goal x: {self.current_pose.x}, y: {self.current_pose.y} has been reached")
            self.get_logger().info(f"The Goal has been reached")
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
            #self.get_logger().info(f"The Goal x: {self.current_pose.x}, y: {self.current_pose.y} has been reached")
            self.get_logger().info(f"The Goal has been reached")
        else:
            self.get_logger().info(f"Distance: {linear_error}, Alpha: {angular_error}")

        self.myPub.publish(msg=msg)

def main(args=None):
    rclpy.init(args=args) # to start ros2 communication

    node = goToNode()
    rclpy.spin(node)

    rclpy.shutdown() # to end ros2 communication

