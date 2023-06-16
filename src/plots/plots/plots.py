import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt

from custom_interfaces.msg import PointArray, ConeArray

class Plots(Node):
    def __init__(self):
        super().__init__('plots')

        self.perception_color = "r"
        self.map_color = "b"
        self.path_color = "y"

        self.perception_points = []
        self.map_points = []

        self.perception_subscription = self.create_subscription(
            ConeArray,
            'perception/cone_coordinates',
            self.plot_perception_callback,
            10
        )

        self.map_subscription = self.create_subscription(
            ConeArray,
            'track_map',
            self.plot_map_callback,
            10
        )

        self.path_subscription = self.create_subscription(
            PointArray,
            'planning_local',
            self.plot_path_callback,
            10
        )

        self.timer = self.create_timer(0.5, self.timer_callback)

    def plot_perception_callback(self, msg):
        for cone in msg.cone_array:
            self.perception_points.append([cone.position.x, cone.position.y,
            self.perception_color])

    def plot_map_callback(self, msg):
        for cone in msg.cone_array:
            self.map_points.append([cone.position.x, cone.position.y, self.map_color])

    def plot_path_callback(self, msg):
        for point in msg.points:
            self.map_points.append([point.x, point.y, self.path_color])

    def timer_callback(self):
        self.plot_points()
        self.map_points = []
        self.perception_points = []

    def plot_points(self):
        if len(plt.get_fignums()) > 0:
            fig = plt.gcf()
            if len(fig.axes) > 0:
                ax1, ax2 = fig.axes
            else:
                ax1, ax2 = fig.subplots(nrows=1, ncols=2)
        else:
            fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2, figsize=(8, 4))

        fig.suptitle("Cones' coordinates")

        ax1.set_title("Perception")
        for x, y, color in self.perception_points:
            ax1.scatter(x, y, c=color)

        ax2.set_title("Map")
        for x, y, color in self.map_points:
            ax2.scatter(x, y, c=color)

        plt.draw()

def main(args=None):
    rclpy.init(args=args)
    plots = Plots()
    rclpy.spin(plots)
    plots.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
