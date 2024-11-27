import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class BagNode(Node):
    def __init__(self, ground_truth_file_1, ground_truth_file_2):
        super().__init__("bag_node")

        self.ground_truth_data_blue = self.load_ground_truth_data(
            ground_truth_file_1, "blue"
        )
        self.ground_truth_data_yellow = self.load_ground_truth_data(
            ground_truth_file_2, "yellow"
        )

        self.ground_truth_data_pub = self.create_publisher(
            MarkerArray, "ground_truth_data", 10
        )

        self.timer = self.create_timer(1.0, self.publish_ground_truth)

    def load_ground_truth_data(self, file_path, label):
        """Load ground truth data from a file, skipping the header line and assigning each point a label."""
        ground_truth_data = []
        try:
            with open(file_path, "r") as file:
                next(file)
                for line in file:
                    x, y = map(float, line.strip().split()[:2])
                    ground_truth_data.append((x, y, label))
            self.get_logger().info(f"Loaded ground truth data for {label}.")
        except Exception as e:
            self.get_logger().error(
                f"Failed to load ground truth data for {label}: {e}"
            )
        return ground_truth_data

    def publish_ground_truth(self):
        """Publish the combined ground truth data as a single MarkerArray message."""
        marker_array = MarkerArray()
        id_counter = 0

        for x, y, label in self.ground_truth_data_blue + self.ground_truth_data_yellow:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.id = id_counter
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.5
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.3

            if label == "blue":
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            marker.color.a = 1.0
            marker_array.markers.append(marker)
            id_counter += 1

        self.ground_truth_data_pub.publish(marker_array)
        self.get_logger().info("Published combined ground truth data as MarkerArray.")


def main(args=None):
    rclpy.init(args=args)
    ground_truth_file_1 = "/home/ws/src/rosbag_groundtruth/ground_truth_data_1_dv4.txt"
    ground_truth_file_2 = "/home/ws/src/rosbag_groundtruth/ground_truth_data_2_dv4.txt"
    node = BagNode(ground_truth_file_1, ground_truth_file_2)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
