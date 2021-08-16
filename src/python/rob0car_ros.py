#! /usr/bin/env python3
import pigpio
import os
import logging
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from motor import Motor
from ROB0Car import ROB0Car

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'steering',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        self.rob0car = ROB0Car()

    def listener_callback(self, msg):
        steering_dc = msg.data
        logging.debug("Steering DC: " + str(steering_dc))
        self.rob0car.steering.set_steering_dc(steering_dc)

def main(args=None):
    # initialize the rob0car node
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    # wait for incoming commands
    rclpy.spin(minimal_subscriber)

    # Interrupt detected, shut down
    minimal_subscriber.rob0car.stop()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()