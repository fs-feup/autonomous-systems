import rclpy
from rclpy.node import Node

from custom_interfaces.msg import PointArray
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion
from .utils import *

STEER_CONTROL = 1
SPEED_CONTROL = 1
KEEP_GOING = 0

class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
        self.control_info = (STEER_CONTROL, SPEED_CONTROL, KEEP_GOING)
        self.steering_angle_velocity = 0.
        self.accelaration = 0
        self.old_error = 0
        self.path = None
        
        self.path_subscription = self.create_subscription(
            PointArray,
            'path_topic',
            self.path_callback,
            10
        )
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.odometry_callback,
            10
        )
        
        # prevent unused variable warning
        self.path_subscription
        self.odom_subscription
    
        self.publisher_ = self.create_publisher(
            AckermannDriveStamped,
            '/cmd', 
            10)
    
        # We will publish a message every 0.5 seconds
        timer_period = 0.5  # seconds
    
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.5 seconds.
        """
        # Create a String message
        ack_msg = AckermannDriveStamped()

        # Set the String message's data
        ack_msg.drive.steering_angle = min(self.steering_angle_velocity, 30.0)
        # ack_msg.drive.steering_angle_velocity = self.steering_angle_velocity

        ack_msg.drive.acceleration = 1.0
        
        # Publish the message to the topic
        self.publisher_.publish(ack_msg)
        
        # Display the message on the console
        self.get_logger().info('publishing')


    def odometry_callback(self, msg):
        self.get_logger().info("Received odom!")
        if self.path is None: return

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

        # Converts quartenions base to euler's base, and updates the class' attributes
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        coords, closest_index = get_closest_point(
            (position.x, position.y, yaw),
            self.path,
        )

        pos_error = right_or_left((position.x, position.y, yaw), coords)

        self.steer(pos_error)

        self.old_error = pos_error
        

    def path_callback(self, path):
        self.get_logger().info("Received path!")
        self.path = []
        
        for point in path.points:
            self.path.append([point.x, point.y])


    def steer(self, error):
        kp = 0.03
        kd = 0.0

        steer_angle_rate = kp*min(error, 10000000) + kd*(error - self.old_error)

        self.old_error = error

        self.steering_angle_velocity = float(steer_angle_rate)


def main(args=None):
    rclpy.init(args=args)

    control_node = ControlNode()

    rclpy.spin(control_node)

    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()