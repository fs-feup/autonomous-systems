import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
import math
import copy
import time # <--- NEW: For timing
from collections import deque

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

# Custom Interfaces
from custom_interfaces.msg import PerceptionOutput

class ConeSlamNode(Node):
    def __init__(self):
        super().__init__('cone_slam_node')

        # --- TUNING ---
        self.declare_parameter('cone_association_dist', 1.2) 
        self.declare_parameter('max_icp_distance', 2.0)
        self.declare_parameter('velocity_smoothing', 0.3)
        
        # --- SUBS/PUBS ---
        self.sub_cones = self.create_subscription(
            PerceptionOutput, '/perception/cones', self.cone_callback, 10)
        
        self.odom_publisher = self.create_publisher(Odometry, '/lidar_odom', 10)
        self.vis_publisher = self.create_publisher(MarkerArray, '/lidar_odom/trajectory', 10)
        self.map_publisher = self.create_publisher(MarkerArray, '/lidar_slam/global_map', 10)
        
        # --- STATE ---
        self.global_map = o3d.geometry.PointCloud()
        self.global_pose = np.identity(4)
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        
        self.prev_time = None
        self.trajectory_history = []
        
        self.get_logger().info("Cone SLAM (Timed) Started")

    def cone_callback(self, msg):
        # --- START TIMER ---
        t0 = time.perf_counter()
        
        current_cones_local = self.extract_cones_to_o3d(msg)
        if len(current_cones_local.points) < 2: return

        curr_time = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) - msg.exec_time
        
        if self.prev_time is None:
            self.prev_time = curr_time
            self.update_global_map(current_cones_local, self.global_pose)
            return

        dt = curr_time - self.prev_time
        if dt <= 0.001: return 

        # --- STEP 1: PREDICTION ---
        vx, vy, wz = self.current_velocity
        dx = vx * dt; dy = vy * dt; d_theta = wz * dt
        c = math.cos(d_theta); s = math.sin(d_theta)
        
        T_motion = np.array([[c, -s, 0, dx], [s, c, 0, dy], [0, 0, 1, 0], [0, 0, 0, 1]])
        T_guess = np.dot(self.global_pose, T_motion)

        # --- STEP 2: REGISTRATION ---
        threshold = self.get_parameter('max_icp_distance').value
        reg = o3d.pipelines.registration.registration_icp(
            current_cones_local, self.global_map, threshold, T_guess,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
        )
        
        T_new = reg.transformation

        # --- STEP 3: UPDATE ---
        T_old_inv = np.linalg.inv(self.global_pose)
        T_diff = np.dot(T_old_inv, T_new)
        
        measured_vx = T_diff[0, 3] / dt
        measured_vy = T_diff[1, 3] / dt
        measured_wz = math.atan2(T_diff[1, 0], T_diff[0, 0]) / dt
        
        speed = math.sqrt(measured_vx**2 + measured_vy**2)

        if speed > 40.0:
            self.get_logger().warn(f"Jump ({speed:.1f}). Coasting.")
            self.global_pose = T_guess
        else:
            self.global_pose = T_new
            alpha = self.get_parameter('velocity_smoothing').value
            if abs(measured_vy) < 0.5: measured_vy *= 0.1
            
            self.current_velocity[0] = (1.0 - alpha) * measured_vx + alpha * self.current_velocity[0]
            self.current_velocity[1] = (1.0 - alpha) * measured_vy + alpha * self.current_velocity[1]
            self.current_velocity[2] = (1.0 - alpha) * measured_wz + alpha * self.current_velocity[2]
            
            self.update_global_map(current_cones_local, self.global_pose)

        # --- PUBLISH ---
        self.publish_odometry(self.current_velocity[0], self.current_velocity[1], self.current_velocity[2], msg.header)
        
        tx = self.global_pose[0, 3]; ty = self.global_pose[1, 3]
        self.publish_trajectory(tx, ty, msg.header)
        
        # Publish Map every 5 frames
        if msg.header.stamp.nanosec % 5 == 0:
            self.publish_global_map(msg.header)

        self.prev_time = curr_time

        # --- END TIMER & PRINT ---
        t1 = time.perf_counter()
        exec_ms = (t1 - t0) * 1000.0
        self.get_logger().info(f"SLAM Step: {exec_ms:.2f} ms | Map Size: {len(self.global_map.points)} cones")

    def update_global_map(self, local_cones, pose):
        cones_global = copy.deepcopy(local_cones)
        cones_global.transform(pose)
        new_points = np.asarray(cones_global.points)
        existing = np.asarray(self.global_map.points)
        
        if len(existing) == 0:
            self.global_map.points = o3d.utility.Vector3dVector(new_points)
            return

        pcd_tree = o3d.geometry.KDTreeFlann(self.global_map)
        assoc_dist = self.get_parameter('cone_association_dist').value
        cones_to_add = []
        
        for p in new_points:
            [k, _, _] = pcd_tree.search_radius_vector_3d(p, assoc_dist)
            if k == 0: cones_to_add.append(p)
        
        if len(cones_to_add) > 0:
            all_points = np.vstack((existing, np.array(cones_to_add)))
            self.global_map.points = o3d.utility.Vector3dVector(all_points)

    def extract_cones_to_o3d(self, msg):
        cones = msg.cones.cone_array
        pts = []
        for c in cones:
            if (c.position.x**2 + c.position.y**2) < 225.0:
                pts.append([c.position.x, c.position.y, 0.0])
        pcd = o3d.geometry.PointCloud()
        if len(pts)>0: pcd.points = o3d.utility.Vector3dVector(np.array(pts, dtype=np.float64))
        return pcd

    def publish_trajectory(self, x, y, header):
        p = Point(); p.x = x; p.y = y; p.z = 0.0
        self.trajectory_history.append(p)
        if len(self.trajectory_history) > 3000: self.trajectory_history.pop(0)

        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = "map"; m.header.stamp = header.stamp
        m.ns = "traj"; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.2; 
        m.color.a = 1.0; m.color.g = 1.0; m.points = self.trajectory_history
        ma.markers.append(m)
        self.vis_publisher.publish(ma)

    def publish_global_map(self, header):
        points = np.asarray(self.global_map.points)
        if len(points) == 0: return

        marker_array = MarkerArray()
        for i, p in enumerate(points):
            m = Marker()
            m.header.frame_id = "map"; m.header.stamp = header.stamp
            m.ns = "global_cones"; m.id = i
            m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position.x = p[0]; m.pose.position.y = p[1]; m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = 0.25; m.scale.y = 0.25; m.scale.z = 0.30
            m.color.a = 1.0; m.color.r = 1.0; m.color.g = 0.55; m.color.b = 0.0
            marker_array.markers.append(m)
            
        self.map_publisher.publish(marker_array)

    def publish_odometry(self, vx, vy, wz, header):
        odom = Odometry()
        odom.header.stamp = header.stamp
        odom.header.frame_id = "map"; odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx; odom.twist.twist.linear.y = vy; odom.twist.twist.angular.z = wz
        odom.twist.covariance = [0.01]*36 
        self.odom_publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = ConeSlamNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()