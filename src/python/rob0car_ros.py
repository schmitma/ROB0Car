#! /usr/bin/env python3
import pigpio
import os
import logging
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from motor import Motor
from steering import Steering

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

class ROB0Car(Node):
    def __init__ (self):
        logging.debug("ROBOCar.__init__")

        super().__init__('ROB0Car')

        self._leftMotor = Motor(4, 50)
        self._rightMotor = Motor(5, 50)
        self._steering = Steering(3)

        self._joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self._joy_callback,
            5)

    def drive(self, speed):
        logging.debug("ROBOCar.drive")
        self._leftMotor.set_motor_speed(speed)
        self._rightMotor.set_motor_speed(speed)

    def steer(self, steering_dc):
        self._steering.set_steering_dc(steering_dc)

    def _joy_callback(self, msg):
        self._steering.set_steering_perc(msg.axes[0]*  100)
        motor_speed_perc = msg.axes[1] * 100
        self._leftMotor.set_motor_speed(motor_speed_perc)
        self._rightMotor.set_motor_speed(motor_speed_perc)

        if not self._leftMotor._isArmed and msg.buttons[0]:
            self._leftMotor.arm

        if not self._rightMotor._isArmed and msg.buttons[0]:
            self._rightMotor.arm

def main(args=None):
    logging.debug("main")
    # initialize the rob0car node
    rclpy.init(args=args)
    rob0car = ROB0Car()

    # wait for incoming commands
    logging.debug("main: Waiting for incoming ROS2 messages on topic joy.")
    rclpy.spin(rob0car)

    # Interrupt detected, shut down
    #rob0car.stop()
    rob0car.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()