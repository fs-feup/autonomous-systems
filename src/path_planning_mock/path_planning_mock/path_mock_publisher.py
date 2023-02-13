import rclpy
from rclpy.node import Node

from custom_interfaces.msg import Point2d as Point


class PathMockPublisher(Node):

    def __init__(self):
        super().__init__('path_mock_publisher')
        self.publisher_ = self.create_publisher(Point, 'path_mock', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.

    def timer_callback(self):
        msg = Point()
        msg.x = self.i
        msg.y = self.i + 1.0
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: X:{msg.x}, Y:{msg.y}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    path_mock_publisher = PathMockPublisher()

    rclpy.spin(path_mock_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_mock_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()