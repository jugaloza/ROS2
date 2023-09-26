#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.srv import CatchTurtle
from functools import partial

class turtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.pose_ = None
        self.turtle_to_catch_ = None
        self.get_logger().info("turtle_controller has been started")
        self.subscriber_ = self.create_subscription(Pose,"turtle1/pose",self.callback_pose,10)
        self.publisher_  = self.create_publisher(Twist,"turtle1/cmd_vel",10)
        self.control_loop_timer_ = self.create_timer(0.01,self.control_loop)
        self.turtle_subscriber_ = self.create_subscription(TurtleArray,"alive_turtles",self.callback_alive,10)
        

        

    def callback_pose(self,msg):
        self.pose_ = msg

    def callback_alive(self,msg):
        if len(msg.turtles)> 0:
            self.turtle_to_catch_ = msg.turtles[0]


    def call_catch_turtle(self,name):

        client = self.create_client(CatchTurtle,"catch_turtle")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Unable to connect server")

        
        request = CatchTurtle.Request()

        request.name = name

        future = client.call_async(request)

        future.add_done_callback(partial(self.callback_catch,name=name))

        

        
    def callback_catch(self,future,name):
        try:
            response = future.result()
            
            if not response.success:
                self.get_logger().info(f"turtle {name} c cannot be caught. ")
        except Exception as e:
            self.get_logger().error("Error while getting response")
    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch_ == None:
            return
        
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        msg = Twist()

        if distance > 0.5:
            msg.linear.x = 2 * distance

            goal_theta = math.atan2(dist_y,dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
            msg.angular.z = 6 * diff
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_catch_turtle(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None
        

        self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = turtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()        

if __name__ == "__main__":
    main()


