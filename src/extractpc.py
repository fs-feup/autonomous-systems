import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points


class PointCloudExtractor(Node):
    def __init__(self, topic_name, output_file):
        super().__init__("pointcloud_extractor")
        self.subscription = self.create_subscription(
            PointCloud2, topic_name, self.listener_callback, 10
        )
        self.output_file = output_file
        self.extracted = False

    def listener_callback(self, msg):
        if not self.extracted:
            self.get_logger().info(f"Extracting point cloud data...")

            # Extract points from PointCloud2 message
            points = list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

            # Write points to PCD file
            self.save_pcd(points)

            self.get_logger().info(f"Point cloud saved to {self.output_file}")
            self.extracted = True
            rclpy.shutdown()

    def save_pcd(self, points):
        """Save points to a .pcd file in ASCII format."""
        with open(self.output_file, "w") as f:
            # Write PCD file header
            f.write(f"# .PCD v0.7 - Point Cloud Data file format\n")
            f.write(f"VERSION 0.7\n")
            f.write(f"FIELDS x y z intensity\n")
            f.write(f"SIZE 4 4 4 4\n")
            f.write(f"TYPE F F F F\n")
            f.write(f"COUNT 1 1 1 1\n")
            f.write(f"WIDTH {len(points)}\n")
            f.write(f"HEIGHT 1\n")
            f.write(f"VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\n")
            f.write(f"DATA ascii\n")

            # Write point data
            for point in points:
                f.write(f"{point[0]} {point[1]} {point[2]} {0}\n")


def main(args=None):
    rclpy.init(args=args)
    topic_name = "/lidar_points"  # Replace with your topic name
    output_file = "src/perception/test/point_clouds/accelaration_start.pcd"  # Desired output file name
    node = PointCloudExtractor(topic_name, output_file)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
