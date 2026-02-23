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
w/s:    increase/decrease acceleration
a/d:    increase/decrease steering angle
q:      quit
CTRL-C to force quit
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
            msg.throttle_fl = 0.0
            msg.throttle_fr = 0.0
            msg.throttle_rl = acc
            msg.throttle_rr = acc
            msg.steering = steer
            self.publisher.publish(msg)
            time.sleep(self.timeout)
        # Publish stop message when thread exits
        msg = ControlCommand()
        msg.throttle_fl = 0.0
        msg.throttle_fr = 0.0
        msg.throttle_rl = 0.0
        msg.throttle_rr = 0.0
        msg.steering = 0.0
        self.publisher.publish(msg)


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
    acceleration = 0.0
    steering = 0.0
    update_needed = True
    try:
        while 1:
            key = get_key(settings, 0.1)
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
