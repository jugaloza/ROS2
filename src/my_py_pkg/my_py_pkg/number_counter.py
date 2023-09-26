#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64,String
from example_interfaces.srv import SetBool

class NumberCounterNode(Node):

    def __init__(self):

        self.number_ = 0
        super().__init__("Number_counter")
        self.get_logger().info("Number counter has been created")
        self.subscriber_ = self.create_subscription(Int64,"number",self.callback_subscriber,10)
        self.publisher_ = self.create_publisher(Int64,"number_count",10)
        self.serivce_ = self.create_service(SetBool,"reset_counter",self.callback_set_bool)
        self.create_timer(0.5,self.callback_publisher)

    def callback_set_bool(self,request,response):

        if request.data:
            self.number_ = 0
            response.success = True
            response.message = "Counter set to 0"
            #self.get_logger().info("Counter set to  0 ")

        else:
            response.success = False
            response.message = "Counter not set to 0"

        return response


    def callback_subscriber(self,msg):

        #data = "data : " + str(msg.data)
        
        #self.get_logger().info(data)
        self.number_ = msg.data

    def callback_publisher(self):
        msg = Int64()
        msg.data = self.number_
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()