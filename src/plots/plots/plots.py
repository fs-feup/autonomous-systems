import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib
import math
import time
from os import system, name

from custom_interfaces.msg import PointArray, ConeArray, Pose as PoseMsg
from eufs_msgs.msg import ConeArrayWithCovariance, CarState

"""!
@brief Function to clear the terminal.
"""
def clear():
   # for windows
   if name == 'nt':
      _ = system('cls')

   # for mac and linux
   else:
    _ = system('clear')


class Position:
    def __init__(self, x: float, y: float):
        self._x: float = x
        self._y: float = y

    @property
    def x(self) -> float:
        return self._x
    
    @x.setter
    def x(self, x: float):
        self._x = x
    
    @property
    def y(self) -> float:
        return self._y
    
    @y.setter
    def y(self, y: float):
        self._y = y

    def distance(self, other: 'Position') -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)


class Pose(Position):
    def __init__(self, x: float, y: float, orientation: float):
        super().__init__(x, y)
        self._orientation = orientation

    @property
    def orientation(self) -> float:
        return self._orientation
    
    @orientation.setter
    def orientation(self, orientation: float):
        self._orientation = orientation


class Cone(Position):
    def __init__(self, x: float, y: float, color: str):
        super().__init__(x, y)
        self._color = color

    @property
    def color(self) -> str:
        return self._color
    
    @color.setter
    def color(self, color: str):
        self._color = color

"""!
@brief Function to convert relative coordinates to absolute coordinates.
@param relative_coordinates Relative coordinates from perception.
@param localization Localization of the vehicle.
@return Absolute coordinates.
"""
def relative_to_absolute_coordinates(relative_coordinates: Position,
                                      localization: Pose) -> Position:
    translation = (localization.x, localization.y)
    rotation_angle = localization.orientation
    x = math.cos(rotation_angle) * relative_coordinates.x - math.sin(
        rotation_angle) * relative_coordinates.y + translation[0]
    y = math.sin(rotation_angle) * relative_coordinates.x + math.cos(
        rotation_angle) * relative_coordinates.y + translation[1]
    return Position(x, y)


class Plots(Node):
    map_color_dict = {"orange_cone": "#d07413", "blue_cone": "#0b5394",
                       "yellow_cone": "#f1c232",
                         "large_orange_cone": "#990000"}
    true_map_color_dict = {"orange_cone": "#f6b26b", "blue_cone": "#2986cc",
                            "yellow_cone": "#d07413",
                              "large_orange_cone": "#e06666"}

    def __init__(self):
        super().__init__('plots')

        # Initialize variables
        self.perception_color = "r"
        self.path_color = "y"
        self.x_bounds = (-50, 50)
        self.y_bounds = (-50, 50)
        self.adjusted_bounds_mapping: bool = False
        self.localization: Pose = Pose(0, 0, 0)
        self.true_localization: Pose = Pose(0, 0, 0)
        self.perception_points: list(Cone) = []
        self.true_perception_points: list(Cone) = []
        self.map_points: list(Cone) = []
        self.true_map_points: list(Cone) = []
        self.last_statistic: dict = {"max_displacement": 0, "total_displacement": 0, 
                "min_displacement": 0, "avg_dispacement": 0,
                "localization_displacement": 0, "cone_number_difference": 0}
        self.statistics: list = []
        self.start_time = time.time()

        # Subscriptions
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
        self.localization_subscription = self.create_subscription(
            PoseMsg,
            'vehicle_localization',
            self.plot_localization_callback,
            10
        )
        self.true_localization_subscription = self.create_subscription(
            CarState,
            'ground_truth/state',
            self.plot_true_localization_callback,
            10
        )
        self.true_map_points_subscription = self.create_subscription(
            ConeArrayWithCovariance,
            'ground_truth/track',
            self.plot_true_map_callback,
            10
        )
        self.true_perception_points_subscription = self.create_subscription(
            ConeArrayWithCovariance,
            'ground_truth/cones',
            self.true_perception_points_callback,
            10
        )

        # Plots
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(
            nrows=1, ncols=3, figsize=(15, 5))
        self.fig.canvas.manager.set_window_title("Evaluation")
        self.fig.canvas.mpl_connect('close_event', 
                                    lambda event: self.on_close(event))
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.fig.set_size_inches(15, 5)


    """!
    @brief Callback function for the true perception cones topic.
    @param msg ConeArrayWithCovariance message.
    """
    def true_perception_points_callback(self, msg: ConeArrayWithCovariance):
        for cone in msg.blue_cones:
            self.true_perception_points.append(Cone(-cone.point.y, cone.point.x,
             Plots.true_map_color_dict["blue_cone"]))
        for cone in msg.yellow_cones:
            self.true_perception_points.append(Cone(-cone.point.y, cone.point.x,
             Plots.true_map_color_dict["yellow_cone"]))
        for cone in msg.orange_cones:
            self.true_perception_points.append(Cone(-cone.point.y, cone.point.x,
             Plots.true_map_color_dict["orange_cone"]))
        for cone in msg.big_orange_cones:
            self.true_perception_points.append(Cone(-cone.point.y, cone.point.x,
             Plots.true_map_color_dict["large_orange_cone"]))
        for cone in msg.unknown_color_cones:
            self.true_perception_points.append(Cone(-cone.point.y, cone.point.x,
             "grey"))
            
        
    """!
    @brief Callback function to plot the cones positions given 
    by perception.
    @param msg ConeArray message.
    """
    def plot_perception_callback(self, msg: ConeArray):
        for cone in msg.cone_array:
            self.perception_points.append(Cone(-cone.position.y, cone.position.x,
                                            Plots.map_color_dict[cone.color]))


    """!
    @brief Callback function to plot the map given by the
    EKF SLAM.
    @param msg ConeArray message.
    """
    def plot_map_callback(self, msg: ConeArray):
        for cone in msg.cone_array:
            self.map_points.append(Cone(-cone.position.y, cone.position.x,
                    Plots.map_color_dict[cone.color])) # Rotate axis


    """!
    @brief Callback function to plot the localization given by the
    EKF SLAM.
    @param msg Pose message.
    """
    def plot_localization_callback(self, msg: PoseMsg):
        self.localization = Pose(-msg.position.y, msg.position.x, 
                                 msg.theta) # Rotate axis


    """!
    @brief Callback function to plot the true cones' coordinates given by the
    simulator.
    @param msg ConeArrayWithCovariance message.
    """
    def plot_true_map_callback(self, msg: ConeArrayWithCovariance):
        for cone in msg.blue_cones:
            self.true_map_points.append(Cone(-cone.point.y, cone.point.x,
             Plots.true_map_color_dict["blue_cone"]))
        for cone in msg.yellow_cones:
            self.true_map_points.append(Cone(-cone.point.y, cone.point.x,
             Plots.true_map_color_dict["yellow_cone"]))
        for cone in msg.orange_cones:
            self.true_map_points.append(Cone(-cone.point.y, cone.point.x,
             Plots.true_map_color_dict["orange_cone"]))
        for cone in msg.big_orange_cones:
            self.true_map_points.append(Cone(-cone.point.y, cone.point.x,
             Plots.true_map_color_dict["large_orange_cone"]))
        for cone in msg.unknown_color_cones:
            self.true_map_points.append(Cone(-cone.point.y, cone.point.x,
             "grey"))

        if not self.adjusted_bounds_mapping and len(self.true_map_points) > 0:
            self.adjust_bounds()
            self.adjusted_bounds_mapping = True


    """!
    @brief Callback function to adjust the bounds of the plot.
    """
    def adjust_bounds(self):
        x_coords = [cone.x for cone in self.true_map_points]
        y_coords = [cone.y for cone in self.true_map_points]
        new_x_bounds = (min(x_coords) - 5, max(x_coords) + 5)
        new_y_bounds = (min(y_coords) - 5, max(y_coords) + 5)
        assert new_x_bounds[0] < new_x_bounds[1]
        assert new_y_bounds[0] < new_y_bounds[1]
        self.x_bounds = new_x_bounds
        self.y_bounds = new_y_bounds
        

    """!
    @brief Callback function to plot the true localization given by the
    simulator.
    @param msg CarState message.
    """
    def plot_true_localization_callback(self, msg: CarState):
        self.true_localization = Pose(- msg.pose.pose.position.y,
            msg.pose.pose.position.x, msg.pose.pose.orientation.z) #Rotate axis


    def plot_path_callback(self, msg):
        for point in msg.points:
            self.map_points.append([point.x, point.y, self.path_color])


    """!
    @brief Generate evaluation statistics.
    """
    def generate_statistics(self):
        # Mapping
        cone_number_difference = abs(len(self.true_map_points) - len(self.map_points))
        cone_distances_mapping = [[i.distance(j) for j in self.true_map_points]
                           for i in self.map_points]
        max_displacement_mapping = 0
        min_displacement_mapping = 0
        total_displacement_mapping = 0
        avg_displacement_mapping = 0
        if len(cone_distances_mapping) != 0:
            displacements_by_cone_mapping = [min(distances) for distances in cone_distances_mapping]
            max_displacement_mapping = max(displacements_by_cone_mapping)
            min_displacement_mapping = min(displacements_by_cone_mapping)
            total_displacement_mapping = sum(displacements_by_cone_mapping)
            avg_displacement_mapping = total_displacement_mapping / len(cone_distances_mapping)
        localization_displacement = self.localization.distance(self.true_localization)

        # Perception
        cone_distances_perception = [[i.distance(j) for j in self.true_map_points]
                            for i in self.perception_points]
        max_displacement_perception = 0
        min_displacement_perception = 0
        total_displacement_perception = 0
        avg_displacement_perception = 0
        if len(cone_distances_perception) != 0:
            displacements_by_cone_perception = [min(distances) for distances in cone_distances_perception]
            max_displacement_perception = max(displacements_by_cone_perception)
            min_displacement_perception = min(displacements_by_cone_perception)
            total_displacement_perception = sum(displacements_by_cone_perception)
            avg_displacement_perception = total_displacement_perception / len(cone_distances_perception)
        return {"max_displacement_mapping": max_displacement_mapping, "total_displacement_mapping": total_displacement_mapping, 
                "min_displacement_mapping": min_displacement_mapping, "avg_displacement_mapping": avg_displacement_mapping,
                "localization_displacement": localization_displacement,
                  "cone_number_difference": cone_number_difference, "max_displacement_perception": max_displacement_perception, 
                  "total_displacement_perception": total_displacement_perception, 
                "min_displacement_perception": min_displacement_perception, "avg_displacement_perception": avg_displacement_perception,
                "time": time.time() - self.start_time}


    """!
    @brief Callback function from timer.
    """
    def timer_callback(self):
        self.statistics.append(self.generate_statistics())
        self.last_statistic = self.statistics[-1]
        self.plot_points()
        self.print_statistics()
        self.map_points = []
        self.true_perception_points = []
        self.true_map_points = []
        self.perception_points = []


    """!
    @brief Callback function for clicking the close button.
    """
    def on_close(self, _):
        exit(0)


    """!
    @brief Print the latest statistics.
    """
    def print_statistics(self):
        clear()
        print("Statistics:")
        for statistic in self.last_statistic.items():
            print("{}: {}".format(statistic[0], statistic[1]))


    """!
    @brief Plot the points - plot mapping, perception, etc.
    """
    def plot_points(self):

        self.ax1.cla()
        self.ax2.cla()
        self.ax3.cla()

        # Perception
        self.ax1.set_title("Perception")
        for cone in self.perception_points:
            self.ax1.scatter(cone.x, cone.y, c=cone.color, marker="*")
        for cone in self.true_perception_points:
            self.ax1.scatter(cone.x, cone.y, c=cone.color, marker="*")

        # Mapping
        self.ax2.set_title("Mapping Map")
        for cone in self.true_map_points:
            self.ax2.scatter(cone.x, cone.y, c=cone.color, marker="*")
        for cone in self.map_points:
            self.ax2.scatter(cone.x, cone.y, c=cone.color, marker="*")

        # ax2.annotate(f"({self.localization[0]:.1f}, 
        # {self.localization[1]:.1f})", (self.localization[0],
        # self.localization[1]))
        self.ax2.scatter(self.true_localization.x, self.true_localization.y, 
                         c="grey", marker="^")
        self.ax2.scatter(self.localization.x, self.localization.y, 
                         c="black", marker="^")
        
        # Statistics
        self.ax3.set_title("Statistics")
        print([i["time"] for i in self.statistics])
        self.ax3.plot([i["avg_displacement_mapping"] for i in self.statistics])
        self.ax3.plot([i["max_displacement_mapping"] for i in self.statistics])
        self.ax3.plot([i["avg_displacement_perception"] for i in self.statistics])
        self.ax3.plot([i["localization_displacement"] for i in self.statistics])
        # self.ax3.table(cellText=[[x, y] for (x, y) in self.last_statistic.items()], colLabels=["Metric", "Value"],
        #                 loc="bottom", cellLoc="center", colWidths=[0.5, 0.5])

        self.ax1.set_aspect('equal', adjustable='box')
        self.ax1.set_xlim(self.x_bounds[0], self.x_bounds[1])
        self.ax1.set_ylim(self.y_bounds[0], self.y_bounds[1])
        self.ax2.set_aspect('equal', adjustable='box')
        self.ax2.set_xlim(self.x_bounds[0], self.x_bounds[1])
        self.ax2.set_ylim(self.y_bounds[0], self.y_bounds[1])
        self.ax3.set_xlim(0, 50)
        self.ax3.set_ylim(0, 50)

        plt.draw()
        plt.pause(0.9)


def main(args=None):
    if matplotlib.rcParams['figure.raise_window']:
        matplotlib.rcParams['figure.raise_window'] = False
    rclpy.init(args=args)
    plots = Plots()
    rclpy.spin(plots)
    plots.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
