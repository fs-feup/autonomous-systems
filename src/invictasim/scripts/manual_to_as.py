#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from custom_interfaces.msg import ControlCommand


class ManualThrottleBridge(Node):
    def __init__(self):
        super().__init__("manual_throttle_bridge")

        # Publisher for the simulator's control command
        self.publisher_ = self.create_publisher(ControlCommand, "/control/command", 10)

        # Subscriber for the manual throttle (0.0 to 1.0)
        self.subscription = self.create_subscription(
            Float64, "/vehicle/manual_throttle", self.throttle_callback, 10
        )

        self.get_logger().info("Python Manual Throttle Bridge node has been started.")
        self.get_logger().info(
            "Mapping /vehicle/manual_throttle -> /control/command (Rear-Wheel Drive)"
        )

    def throttle_callback(self, msg):
        # Create the message
        cmd = ControlCommand()

        # Clamp and safety check
        val = max(0.0, min(1.0, float(msg.data * 0.01)))

        # Assign to rear wheels (FSFEUP 02 144s5p powertrain logic)
        cmd.throttle_fl = 0.0
        cmd.throttle_fr = 0.0
        cmd.throttle_rl = val
        cmd.throttle_rr = val

        # Steering remains zero in this simple throttle bridge
        cmd.steering = 0.0

        # Publish to simulator
        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    bridge = ManualThrottleBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
