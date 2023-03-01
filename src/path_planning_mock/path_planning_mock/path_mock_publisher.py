import rclpy
from rclpy.node import Node

from custom_interfaces.msg import PointArray, Point2d

class PathMockPublisher(Node):

    def __init__(self):
        super().__init__('path_mock_publisher')
        self.create_path()

        self.publisher_ = self.create_publisher(PointArray, 'path_mock', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def create_path(self):
        path = PointArray()

        def add_point(x, y):
            point = Point2d(
                x=float(x),
                y=float(y)
            )
            path.points.append(point)

        for i in range(10):
            add_point(0, i)

        for i in range(1, 10):
            add_point(i, 9)

        for i in range(8,-1,-1):
            add_point(9, i)

        for i in range(8,-1,-1):
            add_point(i, 0)

        self.path = path

    def timer_callback(self):
        def path_to_str(path):
            points = map(lambda v: str((v.x, v.y)), path.points)
            return "[ " + ', '.join(points) + " ]"
        self.get_logger().info(f"Msg:\n{path_to_str(self.path)}")
        self.publisher_.publish(self.path)


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