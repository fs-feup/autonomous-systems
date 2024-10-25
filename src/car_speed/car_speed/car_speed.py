import rclpy
from rclpy.node import Node
from pacsim.msg import Wheels
import rclpy.timer
from std_msgs.msg import Float32
from math import pi
from time import sleep


class CarSpeed(Node):

    def __init__(self):

        super().__init__("car_speed")

        self.subscription = self.create_subscription(
            Wheels,
            "/pacsim/wheelspeeds",
            self.listener_callback,
            10,
        )

        self.publisher = self.create_publisher(Float32, "/carspeed/linear", 10)

    def listener_callback(self, msg: Wheels):
        self.wheel_rpm = msg.fl
        linear_v = Float32()
        wheel_radius = 0.26
        linear_v.data = wheel_radius * self.wheel_rpm * (2 * pi) / 60

        self.publisher.publish(linear_v)
        sleep(1)
        self.get_logger().info(f"Published velocity: {linear_v}")


def main(args=None):
    rclpy.init(args=args)
    node = CarSpeed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
