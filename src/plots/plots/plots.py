import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

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

        self.fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2, figsize=(8, 4))
        ax1.set_title("Perception")
        ax2.set_title("Map + Planning")

        self.scatter1 = ax1.scatter([], [], c=self.perception_color)
        self.scatter2 = ax2.scatter([], [], c=self.map_color)

        anim = FuncAnimation(self.fig, self.plot_points, frames=range(100), interval=200)
        plt.show()

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

    def plot_points(self, frame):
        p_x = []
        p_y = []
        for x, y, _ in self.perception_points:
            p_x.append(x)
            p_y.append(y)
        self.scatter1.set_offsets(np.column_stack((p_x, p_y)))

        m_x = []
        m_y = []
        for x, y, _ in self.map_points:
            m_x.append(x)
            m_y.append(y)
        self.scatter2.set_offsets(np.column_stack((m_x, m_y)))


def main(args=None):
    rclpy.init(args=args)
    plots = Plots()
    rclpy.spin(plots)
    plots.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    