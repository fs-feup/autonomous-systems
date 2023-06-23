import rclpy
from rclpy.node import Node
import numpy as np
import math

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
        self.track_path = '/home/vitor/Downloads/skidpadpath.txt'
        # self.create_path(straight)
        self.create_path2()

        self.publisher_ = self.create_publisher(PointArray, 'path_mock', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def create_path(self, straight):
        """!
        @brief Creates path
        @param self The object pointer
        """
        path = PointArray()
        delta_l = 3.5

        def add_point(x, y):
            point = Point2d(
                x=float(x),
                y=float(y)
            )
            path.points.append(point) if point not in path.points else None

        if straight:
            # straight path
            for i in np.arange(0, 100, delta_l):
                add_point(i,-10)
            self.path = path
            return

        chunck_begging = 0
        length = 36
        n_points = length/ delta_l + 1
        chunck_ending = chunck_begging + length - 1
        for i in np.arange(chunck_begging, chunck_ending, delta_l):
            add_point(i, 0)

        chunck_begging += length
        length = 4
        n_points = int(math.sqrt(2)*length/ delta_l) + 1
        chunck_ending = chunck_begging + length - 1
        for i in np.linspace(chunck_begging, chunck_ending, n_points)[:-1]:
            add_point(i, 36-i)

        chunck_begging += length
        length = 6
        n_points = int(math.sqrt(2)*length/ delta_l) + 1
        chunck_ending = chunck_begging + length - 1
        for i in np.linspace(chunck_begging, chunck_ending, n_points)[:-1]:
            add_point(i, 36-i)

        chunck_begging += length
        length = 8
        n_points = int(length/ delta_l) + 1
        chunck_ending = chunck_begging + length - 1
        for i in np.arange(chunck_begging, chunck_ending, delta_l):
            add_point(i, -10)

        chunck_begging += length
        length = 6
        n_points = int(math.sqrt(2)*length/ delta_l) + 1
        chunck_ending = chunck_begging + length - 1
        for i in np.linspace(chunck_begging, chunck_ending, n_points)[:-1]:
            add_point(i, i-64)

        chunck_begging = -4
        length = 8
        n_points = int(math.sqrt(2)*length/ delta_l) + 1
        chunck_ending = chunck_begging + length - 1
        for i in np.arange(chunck_begging, chunck_ending, delta_l):
            add_point(60, i)

        chunck_begging = 60
        length = -6
        n_points = abs(int(math.sqrt(2)*length/ delta_l))+ 1
        chunck_ending = chunck_begging + length - 1
        for i in np.linspace(chunck_begging, chunck_ending, n_points)[:-1]:
            add_point(i, 64-i)

        chunck_begging += length
        length = -8
        n_points = abs(int(math.sqrt(2)*length/ delta_l)) + 1
        chunck_ending = chunck_begging + length + 1
        for i in np.arange(chunck_begging, chunck_ending, -delta_l):
            add_point(i, 10)


        """ 
        chunck_begging = 46
        length = -6
        n_points = abs(int(math.sqrt(2)*length/ delta_l)) + 1
        chunck_ending = chunck_begging + length - 1
        for i in np.linspace(chunck_begging, chunck_ending, n_points)[:-1]:
            add_point(i, i-36)

        chunck_begging = 4
        length = -4
        n_points = int(int(math.sqrt(2)*length/ delta_l) + 1)
        chunck_ending = chunck_begging + length - 1
        for i in np.arange(4, 0, -delta_l):
            add_point(40, i)

         """
        self.path = path

    def create_path2(self):
        path = PointArray()

        def add_point(x, y):
            point = Point2d(
                y=float(x)/1.8246,
                x=float(y)/1.8246 + 14.4
            )
            path.points.append(point) if point not in path.points else None

        with open(self.track_path, 'r') as f:
            for line in f.readlines()[:]:
                coords = line.split(' ')
                add_point(coords[0], coords[1])

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


    '''

    path = PointArray()

    def add_point(x, y):
        point = Point2d(
            x=float(x),
            y=float(y)
        )
        path.points.append(point) if point not in path.points else None

    with open(self.track_path, 'r') as f:
        for line in f.readlines():
            coords = line.split(' ')
            add_point(coords[1], coords[2])

    print(path)
    '''


