#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random
from functools import partial
import math

from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.srv import CatchTurtle
from turtlesim.srv import Kill


class turtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.turtle_counter = 0
        self.turtle_prefix = "turtle_"
        self.alive_turtles_ = []
        self.timer_ = self.create_timer(1.0,self.spawn_new_turtle)
        self.alive_turtle_publisher_ = self.create_publisher(TurtleArray,"alive_turtles",10)
        self.catch_turtle_service_ = self.create_service(CatchTurtle,"catch_turtle",self.call_kill_server)
        

    def call_kill_server(self,request,response):

        client_kill = self.create_client(Kill,"kill")

        while not client_kill.wait_for_service(1):
            self.get_logger().warn("Unable")

        kill_request = Kill.Request()

        kill_request.name = request.name

        future = client_kill.call_async(kill_request)

        future.add_done_callback(partial(self.callback_catched,name=request.name))

        response.success = True

        return response
    
    def callback_catched(self,future,name):
        try:
            future.result()
            for (i,turtle) in enumerate(self.alive_turtles_):
                if (turtle.name == name):
                    del self.alive_turtles_[i]
                    self.publish_alive_turtles()
                    break
            
        except Exception as e:
            self.get_logger().error("Error while killing turtle")


    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtle_publisher_.publish(msg)

    def spawn_new_turtle(self):
        self.turtle_counter += 1
        name = self.turtle_prefix + str(self.turtle_counter)
        x = random.uniform(0.0,11.0)
        y = random.uniform(0.0,11.0)
        theta = random.uniform(0,2 * math.pi)
        self.call_spawn_turtle(name,x,y,theta)




    def call_spawn_turtle(self,name, x, y, theta):

        client = self.create_client(Spawn,"spawn")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Unable to reach spawn server")

        
        request = Spawn.Request()

        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)

        future.add_done_callback(partial(self.callback_spawned,name=name,x=x,y=y,theta=theta))
        
        
    def callback_spawned(self,future,name,x,y,theta):
        try:
            response = future.result()

            if response.name != "":
                self.get_logger().info(f"Turtle {response.name} spawned")

                turtle = Turtle()

                turtle.name = response.name
                turtle.x = x
                turtle.y = y
                turtle.theta = theta

                self.alive_turtles_.append(turtle)
                self.publish_alive_turtles()

                
        except Exception as e:
            self.get_logger().error("Error while getting response ")




def main(args=None):
    rclpy.init(args=args)
    node = turtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

    