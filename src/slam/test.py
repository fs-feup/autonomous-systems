import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from custom_interfaces.msg import Velocities
import math

class LocationPublisher(Node):
    def __init__(self):
        super().__init__('location_publisher')
        self.publisher_ = self.create_publisher(Marker, 'location', 10)
        self.timer = self.create_timer(1.0, self.publish_marker)
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.last_time = 0
        self.subscription = self.create_subscription(
            Velocities,
            '/state_estimation/velocities',
            self.velocities_callback,
            10
        )

    def velocities_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time == 0:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        self.x += msg.velocity_x * dt * math.cos(self.orientation) - msg.velocity_y * dt * math.sin(self.orientation)
        self.y += msg.velocity_x * dt * math.sin(self.orientation) + msg.velocity_y * dt * math.cos(self.orientation)
        self.orientation += msg.angular_velocity * dt
        self.last_time = current_time
        self.publish_marker()

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "location"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(self.orientation / 2.0)
        marker.pose.orientation.w = math.cos(self.orientation / 2.0)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.publisher_.publish(marker)
        self.get_logger().info(f'Current position: x={self.x:.4f}, y={self.y:.4f}, orientation={self.orientation:.4f}')

def main(args=None):
    rclpy.init(args=args)
    node = LocationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()