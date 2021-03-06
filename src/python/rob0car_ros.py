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
from camera import Camera


logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

class ROB0Car(Node):
    def __init__ (self):
        logging.debug("ROBOCar.__init__")

        super().__init__('ROB0Car')

        self._leftMotor = Motor(13, 'PWM')
        self._rightMotor = Motor(5, 'PWM', True)
        self._steering = Steering(23)

        self._camera = Camera()

        self.close_distance = 0.30 # start slowing down when obstacle within 30 cm is detected
        self.stop_distance = 0.10 # stop motion in direction where obstacle within 10 cm has been detected
        self.distance = 400.0

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

    def drive(self, left_motor_speed, right_motor_speed):
        logging.debug("ROBOCar.drive")

        # Add in a governor to cap forward motion when we're about
        # to collide with something (but still backwards motion)
        governor = min(1, max(0, (self.distance - self.stop_distance)) / \
                           (self.close_distance - self.stop_distance))
        
        logging.debug("Distance: " + str(self.distance))
        logging.debug("Governor: " + str(governor))

        if left_motor_speed > 0:
            left_motor_speed *= governor
        if right_motor_speed > 0:
            right_motor_speed *= governor
        
        self._leftMotor.set_motor_speed(left_motor_speed)
        self._rightMotor.set_motor_speed(right_motor_speed)

    def steer(self, steering_dc):
        self._steering.set_steering_dc(steering_dc)

    def _joy_callback(self, msg):
        logging.debug(f'Joystick axes: {msg.axes}')
        logging.debug(f'Joystick buttons: {msg.buttons}')
        self._steering.set_steering_perc(msg.axes[0] * -100)
        motor_speed_perc = msg.axes[1] * 100
        self.drive(motor_speed_perc, motor_speed_perc)

        logging.debug(self._leftMotor._isArmed == False and msg.buttons[0] == 1)
        logging.debug(self._rightMotor._isArmed == False and msg.buttons[0] == 1)
        
        if self._leftMotor._isArmed == False and msg.buttons[0] == 1:
            self._leftMotor.arm()

        if self._rightMotor._isArmed == False and msg.buttons[0] == 1:
            self._rightMotor.arm()

        if msg.buttons[1] == 1:
            self._camera.home()
        
        if msg.axes[3] != 0:
            self._camera.pan(self._camera.pan_angle + msg.axes[3])
        if msg.axes[4] != 0:
            self._camera.tilt(self._camera.tilt_angle + (msg.axes[4] * (-1)))

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