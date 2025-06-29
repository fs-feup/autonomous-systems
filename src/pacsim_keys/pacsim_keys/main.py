#!/usr/bin/env python

from __future__ import print_function
import threading
import sys
from select import select
import termios
import tty
import math

from rclpy.node import Node
import rclpy
from pacsim.msg import StampedScalar

lateral_command_msg = StampedScalar()
longitudinal_command_msg = StampedScalar()

help_msg = """
w/s:    increase/decrease acceleration
a/d:    increase/decrease steering angle
q:      quit
CTRL-C to force quit
"""


class PublishThread(threading.Thread):
    """!
    @brief Thread to publish control commands to the robot
    """

    def __init__(self, rate):
        """!
        @brief Constructor for the PublishThread class

        @param rate: Rate at which to publish control commands
        """
        super(PublishThread, self).__init__()
        self.node = Node("pacsim_keys")
        self.steering_publisher = self.node.create_publisher(
            StampedScalar, "/pacsim/steering_setpoint", 10
        )
        self.acceleration_publisher = self.node.create_publisher(
            StampedScalar, "/pacsim/throttle_setpoint", 10
        )
        self.acceleration: float = 0.0
        self.steering_angle: float = 0.0

        # Condition to lock the thread waiting to be notified
        self.condition = threading.Condition()
        self.new_command = False
        self.done = False

        self.start()

    def update(self, acceleration: float, steering_angle: float):
        """!
        @brief Update the control commands to be published

        @param acceleration: Acceleration value
        @param steering_angle: Steering angle value
        """
        self.condition.acquire()
        self.acceleration = acceleration
        self.steering_angle = steering_angle
        self.condition.notify()
        self.condition.release()

    def stop(self):
        """!
        @brief Stop the thread
        """
        self.done = True
        self.update(0, 0)
        self.join()

    def run(self):
        """!
        @brief Run the thread to publish control commands
        """
        while not self.done:

            # Don't spam commands to the sim
            if not self.new_command:
                continue

            # Lock the condition
            self.condition.acquire()

            lateral_command_msg.value = self.steering_angle
            longitudinal_command_msg.value = self.acceleration

            # Release the condition and publish
            self.condition.release()
            self.acceleration_publisher.publish(longitudinal_command_msg)
            self.steering_publisher.publish(lateral_command_msg)

            self.new_command = False

        # Publish stop message when thread exits
        lateral_command_msg.value = 0.0
        longitudinal_command_msg.value = 0.0
        self.acceleration_publisher.publish(longitudinal_command_msg)
        self.steering_publisher.publish(lateral_command_msg)


def get_key(settings, timeout):
    """!
    @brief Get a single key from the user.
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    """!
    @brief Main function to control the pacsim robot using the keyboard

    @details This function reads the keyboard inputs and updates
    the control commands accordingly, for the other thread to publish them.
    """

    rclpy.init()
    settings = termios.tcgetattr(sys.stdin)

    key_timeout = 0.5
    pub_thread = PublishThread(10000)
    acceleration: float = 0.0
    steering: float = 0.0
    clicked: bool = False
    published: bool = False
    acceleration = 0.0
    steering = 0.0

    try:
        pub_thread.update(acceleration, steering)
        while 1:
            key: str = str(get_key(settings, key_timeout))
            if key == "w":
                pub_thread.new_command = True
                clicked = True
                acceleration += 0.1
                acceleration = min(acceleration, 1.0)
            elif key == "s":
                pub_thread.new_command = True
                clicked = True
                acceleration -= 0.1
                acceleration = max(acceleration, -1.0)
            if key == "a":
                pub_thread.new_command = True
                clicked = True
                steering += math.pi / 32
                steering = min(steering, math.pi / 8)
            elif key == "d":
                pub_thread.new_command = True
                clicked = True
                steering += -math.pi / 32
                steering = max(steering, -math.pi / 8)
            if key == "q":
                break
            if key == "\x03":
                break
            if clicked or published:
                pub_thread.update(acceleration, steering)
            published = clicked
            clicked = False

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
