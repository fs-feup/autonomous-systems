import rclpy
from rclpy.node import Node
import math

from custom_interfaces.msg import PointArray
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion
from .utils import get_closest_point, get_position_error
import numpy as np

STEER_CONTROL = 1
SPEED_CONTROL = 1
KEEP_GOING = 0

class ControlNode(Node):
    """!
    @brief Class that controls the car's steering and speed.
    """

    def __init__(self):
        """!
        @brief Constructor of the class.
        @param self The object pointer.
        """
        super().__init__('control_node')
        
        ## Tuple that contains the control information.
        self.control_info = (STEER_CONTROL, SPEED_CONTROL, KEEP_GOING)
        
        ## Steering angle velocity.
        self.steering_angle_velocity = 0.
        
        ## Acceleration.
        self.lin_speed = 0

        ## Old error.
        self.old_error = 0

        ## Path.
        self.path = None
        
        self.path_subscription = self.create_subscription(
            PointArray,
            'path_mock',
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
        """!
        @brief Controls publisher callback.
        @param self The object pointer.
        """
        # Create a String message
        ack_msg = AckermannDriveStamped()

        # Set the String message's data
        ack_msg.drive.steering_angle = min(self.steering_angle_velocity, 30.0)
        # ack_msg.drive.steering_angle_velocity = self.steering_angle_velocity

        ack_msg.drive.acceleration = 1.0 if self.lin_speed < 1.5 else 0.0
        
        # Publish the message to the topic
        self.publisher_.publish(ack_msg)
        
        # Display the message on the console
        self.get_logger().info('publishing')


    def odometry_callback(self, msg):
        """!
        @brief Odometry callback.
        @param self The object pointer.
        @param msg Odometry message.
        """

        # self.get_logger().info("Received odom!")
        if self.path is None:
            return

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        speed = msg.twist.twist.linear
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]


        self.lin_speed = math.sqrt(speed.x**2 + speed.y**2)

        # Converts quartenions base to euler's base, and updates the class' attributes
        yaw = euler_from_quaternion(orientation_list)[2]

        closest_point = get_closest_point(
            [position.x, position.y],
            self.path,
        )

        # gets position error
        pos_error = get_position_error([position.x, position.y, yaw], closest_point)

        self.steer(pos_error)

        self.old_error = pos_error

        # show info
        self.get_logger().info(f"\n\n\n\nerror: {pos_error}"+
            f"\nsteerin angle velocity: {self.steering_angle_velocity}," + 
            f"\nspeed: {self.lin_speed}" + 
            f"\nclosest: {closest_point}"
            f"\nposition: {(position.x, position.y, yaw)}")
        

    def path_callback(self, path):
        """!
        @brief Path callback.
        @param self The object pointer.
        @param path Path message.
        """
        # self.get_logger().info("Received path!")
        self.path = []
        
        for point in path.points:
            self.path.append([point.x, point.y])
        self.path = np.array(self.path)


    def steer(self, error):
        """!
        @brief Steers the car.
        @param self The object pointer.
        @param error Error.
        """
        kp = 0.3
        kd = 4

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