#! /usr/bin/env python3
import pigpio
import os
import logging
import sys
import rclpy
from rclpy.node import Node
from rob0car_actors.rob0car_actors.camera_pan_tilt import CameraPanTilt
from sensor_msgs.msg import Joy
from rob0car_interfaces.msg import ESC
from motor import Motor
from steering import Steering

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

class ROB0CarActors(Node):
    def __init__ (self):
        logging.debug("ROB0CarActors.__init__()")
 
        super().__init__('rob0car_actors')

        self._leftMotor = Motor(13, 'PWM')
        self._rightMotor = Motor(5, 'PWM', True)
        self._steering = Steering(23)

        self._camera_pan_tilt = CameraPanTilt()

        self._joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self._joy_callback,
            5)
        
        self._esc_subscription = self.create_subscription(
            ESC,
            'ESC',
            self._esc_callback,
            10
        )

    def _joy_callback(self, msg):
        logging.debug("ROB0CarActors._joy_callback()")

        if msg.buttons[1] == 1:
            self._camera_pan_tilt.home()
        
        if msg.axes[3] != 0:
            self._camera_pan_tilt.pan(self._camera_pan_tilt.pan_angle + msg.axes[3])
        if msg.axes[4] != 0:
            self._camera_pan_tilt.tilt(self._camera_pan_tilt.tilt_angle + (msg.axes[4] * (-1)))
        
    def _esc_callback(self, esc_msg):
        logging.debug("ROB0CarActors._esc_callback()")

        if esc_msg.channel == 0:
            self._leftMotor.set_motor_speed(esc_msg.throttle)

            if self._leftMotor._isArmed == False and esc_msg.arm == 1:
                self._leftMotor.arm()
        elif esc_msg.channel == 1:
            self._rightMotor.set_motor_speed(esc_msg.throttle)

            if self._rightMotor._isArmed == False and esc_msg.arm == 1:
                self._rightMotor.arm()

def main(args=None):
    logging.debug("main")
    # initialize the rob0car_actors node
    rclpy.init(args = args)
    rob0car_actors = ROB0CarActors()

    # wait for incoming commands
    logging.debug("main: Waiting for incoming ROS2 messages on topic joy.")
    rclpy.spin(rob0car_actors)

    # Interrupt detected, shut down
    #rob0car.stop()
    rob0car_actors.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
