#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from example_interfaces.srv import AddTwoInts
from functools import partial

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("add_two_ints_client_oop")
        self.call_add_two_ints(6,5)
        self.call_add_two_ints(11,12)

    def call_add_two_ints(self,a,b):

        client = self.create_client(AddTwoInts,"add_two_ints")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Add two Int server to respond")

        request = AddTwoInts.Request()
        request.a = a 
        request.b = b
        
        future = client.call_async(request)

        future.add_done_callback(partial(self.callback_add_two_ints,a=a,b=b))


    def callback_add_two_ints(self,future,a,b):
        try:
            response = future.result()
            self.get_logger().info("Result of " + str(a) + " + " + str(b) + " = " + str(response.sum))

        except Exception as e:
            self.get_logger().error(" Error while getting response from server %s" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
