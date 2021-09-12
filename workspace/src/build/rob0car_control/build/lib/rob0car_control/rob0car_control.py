#! /usr/bin/env python3
import pigpio
import os
import logging
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Range
from rob0car_interfaces.msg import ESC

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

class ROB0CarControl(Node):
    def __init__ (self):
        logging.debug("ROB0CarControl.__init__")
 
        super().__init__('ROB0CarControl')
        self.close_distance = 0.30 # start slowing down when obstacle within 30 cm is detected
        self.stop_distance = 0.10 # stop motion in direction where obstacle within 10 cm has been detected
        self.distance = 400.0

        self.esc_publisher_ = self.create_publisher(ESC, 'ESC', 10)

        self._joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self._joy_callback,
            5)

    def drive(self, left_motor_speed, right_motor_speed, arm):
        logging.debug("ROBOCar.drive")
        
        esc_msg = ESC()
        esc_msg.channel = 0
        esc_msg.throttle = 0
        esc_msg.arm = 0

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
        
        esc_msg.channel = 0
        esc_msg.throttle = left_motor_speed
        esc_msg.arm = arm
        self.esc_publisher_.publish(esc_msg)

        esc_msg.channel = 1
        esc_msg.throttle = right_motor_speed
        esc_msg.arm = arm
        self.esc_publisher_.publish(esc_msg)

    # def steer(self, steering_dc):
    #     self._steering.set_steering_dc(steering_dc)

    def _joy_callback(self, msg):
        logging.debug(f'Joystick axes: {msg.axes}')
        logging.debug(f'Joystick buttons: {msg.buttons}')

        #self._steering.set_steering_perc(msg.axes[0] * -100)
        motor_speed_perc = msg.axes[1] * 100
        self.drive(motor_speed_perc, motor_speed_perc, msg.buttons[0])

def main(args=None):
    logging.debug("main")
    # initialize the rob0car node
    rclpy.init(args=args)
    rob0car_control = ROB0CarControl()

    # wait for incoming commands
    logging.debug("main: Waiting for incoming ROS2 messages on topic joy.")
    rclpy.spin(rob0car_control)

    # Interrupt detected, shut down
    #rob0car.stop()
    rob0car_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
