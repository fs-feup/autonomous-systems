#!/usr/bin/env python

from __future__ import print_function
import threading
import sys
from select import select
import termios
import tty
import math
import time

from rclpy.node import Node
import rclpy
from pacsim.msg import StampedScalar
from pacsim.msg import Wheels

lateral_command_msg = StampedScalar()
longitudinal_command_msg = Wheels()

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
            Wheels, "/pacsim/throttle_setpoint", 10
        )

        self.acceleration: float = 0.0
        self.steering_angle: float = 0.0

        self.condition = threading.Condition()
        self.done = False
        self.timeout = 1.0 / rate
        self.start()

    def update(self, acceleration: float, steering_angle: float):
        """!
        @brief Update the control commands to be published

        @param acceleration: Acceleration value
        @param steering_angle: Steering angle value
        """
        with self.condition:
            self.acceleration = acceleration
            self.steering_angle = steering_angle

    def stop(self):
        self.done = True
        self.join()

    def run(self):
        while not self.done:
            with self.condition:
                acc = self.acceleration
                steer = self.steering_angle
            # Set all wheels to 0 except rear for throttle
            lateral_command_msg.value = steer
            longitudinal_command_msg.fl = 0.0
            longitudinal_command_msg.fr = 0.0
            longitudinal_command_msg.rl = acc
            longitudinal_command_msg.rr = acc
            self.acceleration_publisher.publish(longitudinal_command_msg)
            self.steering_publisher.publish(lateral_command_msg)
            time.sleep(self.timeout)
        # Publish stop message when thread exits
        lateral_command_msg.value = 0.0
        longitudinal_command_msg.fl = 0.0
        longitudinal_command_msg.fr = 0.0
        longitudinal_command_msg.rl = 0.0
        longitudinal_command_msg.rr = 0.0
        self.acceleration_publisher.publish(longitudinal_command_msg)
        self.steering_publisher.publish(lateral_command_msg)


def get_key(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    settings = termios.tcgetattr(sys.stdin)

    print(help_msg)

    pub_thread = PublishThread(20)  # 20 Hz update rate

    acceleration: float = 0.0
    steering: float = 0.0

    update_needed = True

    try:
        while 1:
            key = get_key(settings, 0.1)  # Faster timeout for smoother feel

            if key == "w":
                acceleration += 0.1
                acceleration = min(acceleration, 1.0)
                update_needed = True
            elif key == "s":
                acceleration -= 0.1
                acceleration = max(acceleration, -1.0)
                update_needed = True
            elif key == "a":
                steering += math.pi / 32
                steering = min(steering, math.pi / 8)
                update_needed = True
            elif key == "d":
                steering += -math.pi / 32
                steering = max(steering, -math.pi / 8)
                update_needed = True
            elif key == "q" or key == "\x03":
                break

            if update_needed:
                pub_thread.update(acceleration, steering)
                print(f"\rAccel: {acceleration:.2f} | Steer: {steering:.2f}", end="\n")
                update_needed = False

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
