import rclpy
from rclpy.node import Node
import math

from custom_interfaces.msg import PointArray
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion
from .utils import get_closest_point, get_position_error, get_orientation_error,\
                   get_cte, get_reference_speed, get_speed_error
import numpy as np  

STD_SPEED = 1.5
START_BREAKING_POS = 12
LOOK_AHEAD = 0

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
        
        # Steering angle velocity.
        self.steering_angle = 0.
        
        # Acceleration.
        self.acceleration = 0

        # Old error.
        self.old_error = 0

        # Path.
        self.path = None

        # Reference speeds
        self.speeds = None

        # Task completion
        self.done = False
        
        self.path_subscription = self.create_subscription(
            PointArray,
            'planning_local',
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
        timer_period = 0.2  # seconds
    
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
        # TODO: Set the minimum and maximum steering angle
        ack_msg.drive.steering_angle = self.steering_angle if not self.done else 0.
        # ack_msg.drive.steering_angle = self.steering_angle

        # Set car acceleration
        ack_msg.drive.acceleration = float(self.acceleration) if not self.done else -1.
        
        # Publish the message to the topic
        self.publisher_.publish(ack_msg)
        
        # Display the message on the console
        self.get_logger().info("acc: {}\tsteer: {}"
            .format(ack_msg.drive.steering_angle, ack_msg.drive.acceleration))


    def odometry_callback(self, msg):
        """!
        @brief Odometry callback.
        @param self The object pointer.
        @param msg Odometry message.
        """

        # self.get_logger().info("Received odom!")
        if self.path is None:
            return

        # get pose feedback
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

        # get speed feedback
        speed = msg.twist.twist.linear
        lin_speed = math.sqrt(speed.x**2 + speed.y**2)

        # Converts quartenions base to euler's base, and updates the class' attributes
        yaw = euler_from_quaternion(orientation_list)[2]

        # get postion reference
        closest_point, closest_index = get_closest_point(
            [position.x, position.y],
            self.path,
        )

        # get speed reference
        ref_speed = get_reference_speed(self.speeds, closest_index)

        # gets position error
        pos_error = get_position_error([position.x, position.y, yaw], closest_point)

        # gets orientation error
        yaw_error = get_orientation_error(closest_index, self.path, yaw)

        # gets cross track error
        ct_error = get_cte(closest_index, self.path, [position.x, position.y, yaw])

        # get speed error
        speed_error = get_speed_error(lin_speed, ref_speed)

        self.steer(pos_error, yaw_error, ct_error)
        self.accelerate(speed_error)

        if lin_speed < 0.2 and closest_index == len(self.path) - 1 and not self.done:
            self.done = True

        # show info
        self.get_logger().info(f"\n\n\n\npos error: {pos_error}"+
            f"\nyaw error: {yaw_error}," + 
            f"\ncross track error: {ct_error}," + 
            f"\nsteerin angle velocity: {self.steering_angle}," + 
            f"\nspeed: {lin_speed}," + 
            f"\nclosest: {closest_point},"
            f"\nposition: {(position.x, position.y, yaw)}," +
            f"\ndone: {self.done}")


    def path_callback(self, points_list):
        """!
        @brief Path callback.
        @param self The object pointer.
        @param path Path message.
        """
        self.get_logger().info("[received] {} points".format(len(points_list.points)))
        path = []
        speeds = []
        
        for i, point in enumerate(points_list.points):
            path.append([point.x, point.y])

            dist_from_end = len(points_list.points) - i

            # min -> start breaking or not
            speed = STD_SPEED*min(1, dist_from_end/START_BREAKING_POS)
            speeds.append([speed])

        self.path = np.array(path)
        self.speeds = np.array(speeds)


    def steer(self, pos_error, yaw_error, ct_error):
        """!
        @brief Steers the car.
        @param self The object pointer.
        @param pos_error Position Error.
        @param yaw_error Orientation Error.
        @param ct_error Cross Track Error.
        """

        # PID params
        kp = 0.3
        kd = 8

        # compute global error
        error = 0*pos_error + 0*yaw_error + 1*ct_error

        # calculate steering angle command
        steer_angle_rate = kp*min(error, 10000000) + kd*(error - self.old_error)
        
        # save old error for derivative of error calculation
        self.old_error = error

        # save reference in node's attribute to be accessed by other methods
        self.steering_angle = float(steer_angle_rate)

        # show error
        # self.get_logger().info(f"\ntotal error: {error}")

    def accelerate(self, error):
        """!
        @brief Accelerates the car.
        @param self The object pointer.
        @param speed_error Speed Error.
        """
        # PID params
        kp = 1

        # calculate acceleration command
        acceleration = kp*min(error, 10000000)

        self.acceleration = float(acceleration)


def main(args=None):
    rclpy.init(args=args)

    control_node = ControlNode()

    rclpy.spin(control_node)

    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()