#!/usr/bin/env python

import threading
import sys
from select import select
import termios
import tty
import math
import time
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ControlCommand

help_msg = """
w/s:    increase/decrease acceleration (automatic decay to 0 when released)
a/d:    increase/decrease steering angle
q:      quit
"""


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.node = Node("invictasim_keys")
        self.publisher = self.node.create_publisher(
            ControlCommand, "/control/command", 10
        )
        self.acceleration = 0.0
        self.steering_angle = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.timeout = 1.0 / rate
        self.start()

    def update(self, acceleration, steering_angle):
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
            msg = ControlCommand()
            msg.throttle_rl = acc
            msg.throttle_rr = acc
            msg.steering = steer
            self.publisher.publish(msg)
            time.sleep(self.timeout)


def get_key(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    settings = termios.tcgetattr(sys.stdin)
    pub_thread = PublishThread(50)  # Increased to 50Hz for smoother response

    acceleration = 0.0
    steering = 0.0

    # Tuning parameters
    accel_step = 0.05  # How fast throttle increases per 0.02s
    decay_step = 0.01  # How fast throttle returns to 0
    steer_step = math.pi / 64

    print(help_msg)

    try:
        while rclpy.ok():
            # Very short timeout for high responsiveness
            key = get_key(settings, 0.02)

            if key == "w":
                acceleration = min(acceleration + accel_step, 1.0)
            elif key == "s":
                acceleration = max(acceleration - accel_step, -1.0)
            elif key == "":
                # Automatic decay logic: move toward 0.0 when no key is pressed
                if acceleration > 0:
                    acceleration = max(0.0, acceleration - decay_step)
                elif acceleration < 0:
                    acceleration = min(0.0, acceleration + decay_step)

            if key == "a":
                steering = min(steering + steer_step, math.pi / 8)
            elif key == "d":
                steering = max(steering - steer_step, -math.pi / 8)
            elif key == "q" or key == "\x03":
                break

            pub_thread.update(acceleration, steering)
            print(f"\rAccel: {acceleration:.2f} | Steer: {steering:.2f}  ", end="")

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
