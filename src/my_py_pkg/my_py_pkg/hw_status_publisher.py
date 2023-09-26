#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import HardwareStatus
#from my_robot_interfaces.msg import HardwareStatus


class HwStatusPublisherNode(Node):
    def __init__(self):
        super().__init__("Hardware_status_publisher")
        self.hw_publisher = self.create_publisher(HardwareStatus,"hw_status",10)

        self.timer_ = self.create_timer(0.5,self.callback_publish_hw)
        
        self.get_logger().info("Hardware Status Publisher has been started.")


    def callback_publish_hw(self):

        msg = HardwareStatus()

        msg.temperature = 45
        msg.are_motors_ready = False
        msg.debug_message = "Set AS"

        self.hw_publisher.publish(msg)


        




def main(args=None):
    rclpy.init(args=args)
    node = HwStatusPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
