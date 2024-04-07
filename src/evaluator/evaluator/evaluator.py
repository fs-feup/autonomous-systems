import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PointArray, ConeArray, Pose as PoseMsg
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2




class Cone:

    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.color = color

class Evaluator(Node):

    def __init__(self):
        super().__init__('evaluator')
        self.get_logger().info("Evaluator Node has started")

        self.perception_points = []
        self.perception_ground_truth = []

        self.perception_subscription = self.create_subscription(
            ConeArray,
            'cones',
            self.perception_callback,
            10
        )

        self.point_cloud_callback = self.create_subscription(
            PointCloud2,
            '/hesai/pandar',
            self.point_cloud_callback,
            10
        )

        self.ground_truth_callbak = self.create_subscription(
            MarkerArray,
            '/perception/lidar/vis/cluster_markers',
            self.perception_ground_truth_callback,
            10
        )
    
    def perception_callback(self, msg: ConeArray):
        self.get_logger().info("Perception Received")
        self.perception_points = []
        for cone in msg.cone_array:
            self.perception_points.append(Cone(cone.position.x, cone.position.y, cone.color))
    
    def point_cloud_callback(self, msg: PointCloud2):
        self.get_logger().info("Point Cloud Received")
    
    def perception_ground_truth_callback(self, msg: MarkerArray):
        self.get_logger().info("Perception Ground Truth Received")
        self.perception_ground_truth
        for marker in msg.markers:
            cone_position = marker.pose.position
            x = cone_position.x
            y = cone_position.y
            self.perception_ground_truth.append(Cone(x, y, None))
        



def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()