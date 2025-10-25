import rclpy
from rclpy.node import Node
import data_association as da
import numpy as np
from helpers import ve_ekf as ve
import motion_model as mm
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist
from helpers import pub

class StateEstimation(Node):
    def __init__(self):
        super().__init__('state_estimation_node')
        self.marker_pub = self.create_publisher(MarkerArray, '/state_estimation/visualization_map', 10)
        self.pose_marker_pub = self.create_publisher(Marker, '/state_estimation/pose', 10)
        self.velocity_pub = self.create_publisher(Twist, '/state_estimation/velocities', 10)

        # TODO: subscribe to IMU, WSS and SAS PacsSim topics and synchronize them with message filters and link to their callback
        # TODO:  subscribe to perception topic and link to its callback
        # Resolver (motor rpm encoder) is simulated by taking the average of rear wheel rpms and multiplying by the gear ratio
        # self.imu_subscription = ...
        # self.wss_subscription = ...
        # self.sas_subscription = ...
        # self.perception_subscription = ...
        self.map = np.array([], dtype=float)
        self.pose = np.array([0.0, 0.0, 0.0], dtype=float)  # Initial pose (x, y, theta)
        self.velocity_estimator = ve.ExtendedKalmanFilter()
    
    def perception_callback(self, msg):
        global_observations = np.array([], dtype=float) # TODO: convert msg to correct format and global coordinates
        associations = da.data_association(self.map, global_observations)
        if (2 * len(associations)!= len(global_observations)):
            raise ValueError("Number of associations must match number of observations.")
        for i in range(len(associations)):
            if (associations[i] == -1):
                self.map = np.append(self.map, global_observations[i * 2: i * 2 + 2])
        pub.publish_map_markers(self)

    def sensors_callback(self, imu_msg, wss_msg, sas_msg):
        # TODO: initialize the values below
        ax = 0
        ay = 0
        omega = 0
        fl_wss = 0
        fr_wss = 0
        rl_wss = 0
        rr_wss = 0
        sas = 0
        dt = 0
        
        motor_rpm = 4 * (rl_wss + rr_wss) / 2
        self.velocity_estimator.predict([ax, ay, omega], dt)
        self.velocity_estimator.correct([fl_wss, fr_wss, rl_wss, rr_wss, sas, motor_rpm])
        self.pose = mm.update_pose(self.pose, self.velocity_estimator.get_velocities(), dt)
        pub.publish_pose_marker(self)
        pub.publish_velocity(self)

def main(args=None):
    rclpy.init(args=args)
    node = StateEstimation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()