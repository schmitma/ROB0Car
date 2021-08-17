#! /usr/bin/env python3
import pigpio
import os
import logging
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Range
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

        self.close = 0.30 # start slowing down when obstacle within 30 cm is detected
        self.stop = 0.10 # stop motion in direction where obstacle within 10 cm has been detected

        self._joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self._joy_callback,
            5)

        self._range_subscription = self.create_subscription(
            Range,
            'range',
            self._range_callback,
            5)

    def drive(self, speed):
        logging.debug("ROBOCar.drive")
        self._leftMotor.set_motor_speed(speed)
        self._rightMotor.set_motor_speed(speed)

    def steer(self, steering_dc):
        self._steering.set_steering_dc(steering_dc)

    def _joy_callback(self, msg):
        self._steering.set_steering_perc(msg.axes[0] *  100)
        motor_speed_perc = msg.axes[1] * 100
        self._leftMotor.set_motor_speed(motor_speed_perc)
        self._rightMotor.set_motor_speed(motor_speed_perc)

        logging.debug(self._leftMotor._isArmed == False and msg.buttons[0] == 1)
        logging.debug(self._rightMotor._isArmed == False and msg.buttons[0] == 1)
        
        if self._leftMotor._isArmed == False and msg.buttons[0] == 1:
            self._leftMotor.arm()

        if self._rightMotor._isArmed == False and msg.buttons[0] == 1:
            self._rightMotor.arm()

    def _range_callback(self, msg):
        self.distance = msg.range
        
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