from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2
from fs_msgs.msg import Track, Cone
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import message_filters


class PerceptionAdater:
    def __init__(self, node, point_cloud_topic):
        self.node = node
        self.point_cloud_topic = point_cloud_topic
        self.node.point_cloud_subscription = message_filters.Subscriber(
            self.node,
            PointCloud2,
            point_cloud_topic
        )

class PerceptionAdapterROSBag(PerceptionAdater):

    def __init__(self, node, point_cloud_topic, ground_truth_topic):
        super().__init__(node, point_cloud_topic)

        self.node.ground_truth_subscription = message_filters.Subscriber(
            self.node,
            MarkerArray,
            ground_truth_topic
        )

        self.ts = message_filters.TimeSynchronizer(
                            [self.node.point_cloud_subscription, self.node.perception_subscription], 
                            10)
        
        self.ts.registerCallback(self.callback)

    def callback(self, point_cloud, perception):
        self.node.get_logger().info("Synchronizing")



class PerceptionAdapterFSDS(PerceptionAdater):

    def __init__(self, node, point_cloud_topic):

        super().__init__(node, point_cloud_topic)

        self.track = []

        self.current_odom_position = None
        self.current_odom_orientation = None

        self.node.track_subscription = self.node.create_subscription(
            Track,
            "/fsds/testing_only/track",
            self.track_callback,
            10
        )

        self.node.odometry_subscription = message_filters.Subscriber(
            self.node,
            Odometry,
            "/fsds/testing_only/odom",
        )

        self.ts = message_filters.TimeSynchronizer(
            [self.node.perception_subscription, self.node.point_cloud_subscription, self.node.odometry_subscription], 
            10
        )

        self.ts.registerCallback(self.callback)

    def callback(self, perception, point_cloud, odometry):
        self.node.get_logger().info("Synchronizing")

    def point_cloud_callback(self, msg: PointCloud2):
        self.node.get_logger().info("Point Cloud Received")

    def track_callback(self, msg: Track):
        self.node.get_logger().info("Track Received")
        for cone in msg.track:
            cone_position = cone.location
            x = cone_position.x
            y = cone_position.y
            self.track.append(Cone(x, y, None))

    def odometry_callback(self, msg: Odometry):
        self.get_logger().info("Odometry values Received")
        self.current_odom_position = msg.pose.pose.position
        self.current_odom_orientation = msg.pose.pose.orientation
        self.transform_cones_to_car_frame()

    def transform_cones_to_car_frame(self):
        rotation_matrix = PerceptionAdapterFSDS.quaternion_to_rotation_matrix(self.current_odom_orientation)
        self.node.perception_ground_truth = []
        for cone in self.track:
            cone_position = np.array([cone.x, cone.y, 0.0])
            transformed_position = np.dot(rotation_matrix, cone_position) + np.array([
                self.current_odom_position.x,
                self.current_odom_position.y,
                0.0
            ])
            self.node.perception_ground_truth.append(transformed_position)

    @staticmethod
    def quaternion_to_rotation_matrix(quaternion):
        q0, q1, q2, q3 = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        rot_matrix = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]])
        return rot_matrix
