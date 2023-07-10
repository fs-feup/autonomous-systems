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

        self.fig, (self.ax1, self.ax2) = plt.subplots(nrows=1, ncols=2, figsize=(8, 4))
        self.fig.canvas.manager.set_window_title("Cones' coordinates")

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

    def plot_points(self):
        self.ax1.cla()
        self.ax2.cla()

        for x, y, color in self.perception_points:
            self.ax1.scatter(x, y, c=color)
        self.ax1.set_title("Perception")

        for x, y, color in self.map_points:
            self.ax2.scatter(x, y, c=color)
        self.ax2.set_title("Map + Trajectory")

        self.fig.canvas.draw()
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    plots = Plots()
    rclpy.spin(plots)
    plots.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
