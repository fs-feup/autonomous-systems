from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist
from tf_transformations import quaternion_from_euler

def publish_map_markers(self : Node):
    marker_array = MarkerArray()
    for i in range(len(self.map) // 2):
        x, y = self.map[i * 2], self.map[i * 2 + 1]
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "landmarks"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker_array.markers.append(marker)
    
    self.marker_pub.publish(marker_array)   
        
def publish_pose_marker(self : Node):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = self.get_clock().now().to_msg()
    marker.ns = "pose"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    x, y, theta = self.pose
    marker.pose.position.x = float(x)
    marker.pose.position.y = float(y)
    marker.pose.position.z = 0.0
    q = quaternion_from_euler(0, 0, theta)
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]
    marker.scale.x = 1.0  # Arrow shaft length
    marker.scale.y = 0.2  # Arrow shaft width
    marker.scale.z = 0.2  # Arrow head height
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    self.pose_marker_pub.publish(marker)
        
def publish_velocity(self : Node):
    vel = self.velocity_estimator.get_velocities()  # Expected: [vx, vy, omega]
    twist_msg = Twist()
    twist_msg.linear.x = float(vel[0])  # vx
    twist_msg.linear.y = float(vel[1])  # vy
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = float(vel[2])  # omega
    self.velocity_pub.publish(twist_msg)