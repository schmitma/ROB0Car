#! /usr/bin/env python3
import pigpio
import os
import logging
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from motor import Motor
from ROB0Car import ROB0Car

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

class MinimalSubscriber(Node):
    def __init__(self):
        logging.debug("MinimalSubscriber.__init__")
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'steering',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        self.rob0car = ROB0Car()

    def listener_callback(self, msg):
        logging.debug("MinimalSubscriber.listener_callback")
        command = msg.data
        logging.debug("Command: " + command)

def main(args=None):
    logging.debug("main")
    # initialize the rob0car node
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    # wait for incoming commands
    logging.debug("main: Waiting for incoming ROS2 messages on topic steering.")
    rclpy.spin(minimal_subscriber)

    # Interrupt detected, shut down
    minimal_subscriber.rob0car.stop()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()