import rclpy
from rclpy.node import Node
import numpy as np

from custom_interfaces.msg import PointArray, Point2d


class PathMockPublisher(Node):
    """!
    @brief Class for publishing path mock
    """

    def __init__(self, straight):
        """!
        @brief Constructor sets up publisher, timer and creates path
        @param self The object pointer
        """
        super().__init__('path_mock_publisher')
        self.create_path(straight)

        self.publisher_ = self.create_publisher(PointArray, 'path_mock', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def create_path(self, straight):
        """!
        @brief Creates path
        @param self The object pointer
        """
        path = PointArray()
        delta_l = 0.5

        def add_point(x, y):
            point = Point2d(
                x=float(x),
                y=float(y)
            )
            path.points.append(point) if point not in path.points else None

        if straight:
            # straight path
            for i in np.arange(0, 100, delta_l):
                add_point(i,-5)
            self.path = path
            return

        # square path
        for i in np.arange(0, 37, 1):
            add_point(i, 0)

        for i in np.linspace(36, 41, 12):
            add_point(i, 36-i)

        for i in np.linspace(40, 47, 18):
            add_point(i, 36-i)
            add_point(i, i-36)

        for i in np.arange(46, 55, 0.5):
            add_point(i, -10)
            add_point(i, 10)

        for i in np.linspace(34, 61, 18):
            add_point(i, i-64)
            add_point(i, 64-i)

        for i in np.arange(-8, 8, 0.5):
            add_point(40, i)
            add_point(60, i)

        self.path = path


    def timer_callback(self):
        """!
        @brief Publishes path
        @param self The object pointer
        """
        def path_to_str(path):
            points = map(lambda v: str((v.x, v.y)), path.points)
            return "[ " + ', '.join(points) + " ]"
        self.get_logger().info(f"Msg:\n{path_to_str(self.path)}")
        self.publisher_.publish(self.path)

def main(args=None, straight=False):
    rclpy.init(args=args)

    path_mock_publisher = PathMockPublisher(straight)

    rclpy.spin(path_mock_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_mock_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()