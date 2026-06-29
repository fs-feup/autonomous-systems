import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import yaml


class TrackGenerator(Node):
    def __init__(self, output_filename="planned_track.yaml"):
        super().__init__("track_generator")
        self.output_filename = output_filename

        # Buffers for the cone data
        self.cones = []
        self.received_yellow = False
        self.received_blue = False

        # Subscriptions matching your C++ publisher names
        self.sub_yellow = self.create_subscription(
            MarkerArray, "/path_planning/yellow_cones", self.yellow_callback, 10
        )
        self.sub_blue = self.create_subscription(
            MarkerArray, "/path_planning/blue_cones", self.blue_callback, 10
        )

        self.get_logger().info(
            "Listening for planning markers... please ensure the car is moving/planning."
        )

    def yellow_callback(self, msg):
        if not self.received_yellow:
            for marker in msg.markers:
                # Format: [x, y, "type"]
                self.cones.append(
                    [marker.pose.position.x, marker.pose.position.y, "yellow_cone"]
                )
            self.received_yellow = True
            self.get_logger().info(f"Captured {len(msg.markers)} yellow cones.")
            self.check_and_save()

    def blue_callback(self, msg):
        if not self.received_blue:
            for marker in msg.markers:
                self.cones.append(
                    [marker.pose.position.x, marker.pose.position.y, "blue_cone"]
                )
            self.received_blue = True
            self.get_logger().info(f"Captured {len(msg.markers)} blue cones.")
            self.check_and_save()

    def check_and_save(self):
        # Save once we have data from both "sides"
        if self.received_yellow and self.received_blue:
            self.save_yaml()
            rclpy.shutdown()

    def save_yaml(self):
        # Construct the YAML structure
        data = {"track": {"start_position": [0, 0], "cones": self.cones}}

        with open(self.output_filename, "w") as f:
            # default_flow_style=None keeps the list format [x, y, "name"] on one line
            yaml.dump(data, f, sort_keys=False, default_flow_style=None)

        self.get_logger().info(f"Successfully created: {self.output_filename}")


def main():
    rclpy.init()
    node = TrackGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
