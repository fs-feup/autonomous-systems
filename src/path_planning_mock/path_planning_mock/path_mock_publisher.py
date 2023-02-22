import rclpy
from rclpy.node import Node

from custom_interfaces.msg import PointArray
from geometry_msgs.msg import Point


class PathMockPublisher(Node):

    def __init__(self):
        super().__init__('path_mock_publisher')
        self.create_path()

        self.publisher_ = self.create_publisher(PointArray, 'path_mock', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def create_path(self):
        path = PointArray()
        for i in range(10):
            point = Point()
            point.x = 0.
            point.y = float(i)
            point.z = 0.

            path.points.append(point)
        for i in range(10):
            point = Point()
            point.x = float(i)
            point.y = 9.
            point.z = 0.

            path.points.append(point)
        for i in range(9,-1,-1):
            point = Point()
            point.x = 9.
            point.y = float(i)
            point.z = 0.

            path.points.append(point)
        for i in range(9,-1,-1):
            point = Point()
            point.x = float(i)
            point.y = 0.
            point.z = 0.

            path.points.append(point)
        self.path = path

    def timer_callback(self):
        if self.path is not None:
            self.publisher_.publish(self.path)
        else:
            self.publisher_.publish([])


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