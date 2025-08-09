#!/usr/bin/env python

from __future__ import print_function
import threading
import sys
from select import select
import termios
import tty
import math
import time
import custom_interfaces.msg._control_command

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
        self.node = Node("car_keys")
        self.control_publisher = self.node.create_publisher(
            custom_interfaces.msg._control_command.ControlCommand, "/as_msgs/controls", 10
        )
        self.acceleration: float = 0.0
        self.steering_angle: float = 0.0
        self.rate = rate

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
        self.new_command = True
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
            # Lock the condition and wait for notification
            self.condition.acquire()
            
            # Wait for new command or timeout
            if not self.new_command:
                self.condition.wait(timeout=1.0/self.rate)
            
            if self.done:
                self.condition.release()
                break
                
            # Create and publish message
            message = custom_interfaces.msg._control_command.ControlCommand()
            message.throttle = self.acceleration
            message.steering = self.steering_angle
            
            # Release the condition and publish
            self.condition.release()
            
            self.control_publisher.publish(message)
            self.new_command = False
            
            # Sleep to maintain rate
            time.sleep(1.0/self.rate)

        # Publish stop message when thread exits
        message = custom_interfaces.msg._control_command.ControlCommand()
        message.throttle = 0.0
        message.steering = 0.0
        self.control_publisher.publish(message)


def get_key(settings, timeout):
    """!
    @brief Get a single key from the user.
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_status(acceleration, steering):
    """!
    @brief Print current status without clearing the screen
    """
    # Move cursor to beginning of line and clear it
    sys.stdout.write('\r')
    sys.stdout.write(' ' * 80)  # Clear the line
    sys.stdout.write('\r')
    
    # Print current status
    accel_bar = create_bar(acceleration, -1.0, 1.0, 20)
    steer_bar = create_bar(steering, -0.34, 0.34, 20)
    
    status = f"Throttle: {acceleration:+.2f} {accel_bar} | Steering: {steering:+.2f} {steer_bar}"
    sys.stdout.write(status)
    sys.stdout.flush()


def create_bar(value, min_val, max_val, length):
    """!
    @brief Create a visual bar representation of a value
    """
    # Normalize value to 0-1 range
    normalized = (value - min_val) / (max_val - min_val)
    normalized = max(0, min(1, normalized))  # Clamp to 0-1
    
    # Create bar
    filled = int(normalized * length)
    bar = '█' * filled + '░' * (length - filled)
    
    return f"[{bar}]"


def main():
    """!
    @brief Main function to control the pacsim robot using the keyboard

    @details This function reads the keyboard inputs and updates
    the control commands accordingly, for the other thread to publish them.
    """

    rclpy.init()
    settings = termios.tcgetattr(sys.stdin)

    key_timeout = 0.1
    pub_thread = PublishThread(50)  # 50 Hz publishing rate
    acceleration: float = 0.0
    steering: float = 0.0
    
    # Print initial help and status
    print(help_msg)
    print("=" * 80)
    print_status(acceleration, steering)

    try:
        pub_thread.update(acceleration, steering)
        while True:
            key: str = str(get_key(settings, key_timeout))
            
            command_changed = False
            
            if key == "w":
                acceleration += 0.05
                acceleration = min(acceleration, 1.0)
                command_changed = True
            elif key == "s":
                acceleration -= 0.05
                acceleration = max(acceleration, -1.0)
                command_changed = True
            elif key == "a":
                steering += 0.05
                steering = min(steering, 0.34)
                command_changed = True
            elif key == "d":
                steering -= 0.05
                steering = max(steering, -0.34)
                command_changed = True
            elif key == "q":
                break
            elif key == "\x03":  # Ctrl+C
                break
            
            if command_changed:
                pub_thread.update(acceleration, steering)
                print_status(acceleration, steering)

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        print("\nShutting down...")
        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()


if __name__ == "__main__":
    main()