import rclpy
from rclpy.node import Node
from custom_interfaces.msg import (
    Velocities,
    WheelRPM,
    SteeringAngle,
)
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3Stamped
import message_filters
import math


class VehicleDynamicsPublisher(Node):
    def __init__(self):
        super().__init__("vehicle_dynamics_publisher")
        self.get_logger().info("Vehicle Dynamics Node started")

        velocity_sub = message_filters.Subscriber(
            self, Velocities, "/state_estimation/velocities"
        )
        yaw_rate_sub = message_filters.Subscriber(
            self, Vector3Stamped, "/imu/acceleration"
        )
        steering_angle_sub = message_filters.Subscriber(
            self, SteeringAngle, "/vehicle/bosch_steering_angle"
        )
        angle_sync = message_filters.ApproximateTimeSynchronizer(
            [velocity_sub, yaw_rate_sub, steering_angle_sub], queue_size=10, slop=0.05
        )
        angle_sync.registerCallback(self.slip_angle_callback)

        rl_rpm_sub = message_filters.Subscriber(self, WheelRPM, "/vehicle/rl_rpm")
        rr_rpm_sub = message_filters.Subscriber(self, WheelRPM, "/vehicle/rr_rpm")
        fl_rpm_sub = message_filters.Subscriber(self, WheelRPM, "/vehicle/fl_rpm")
        fr_rpm_sub = message_filters.Subscriber(self, WheelRPM, "/vehicle/fr_rpm")
        rpm_sync = message_filters.ApproximateTimeSynchronizer(
            [rl_rpm_sub, rr_rpm_sub, fl_rpm_sub, fr_rpm_sub], queue_size=10, slop=0.05
        )
        rpm_sync.registerCallback(self.slip_ratio_callback)

        self.slip_angle_pub = self.create_publisher(
            Float64MultiArray, "/vehicle/slip_angles", 10
        )
        self.slip_ratio_pub = self.create_publisher(
            Float64MultiArray, "/as_msgs/controls", 10
        )

        self.last_yaw = None
        self.last_time = None

    def slip_angle_callback(self, velocities_msg, yaw_rate_msg, steering_angle_msg):
        """
        Callback function for synchronized velocity, yaw_rate, and steering angle data.
        Computes slip angles for the vehicle's center of gravity and all four wheels.

        Args:
            velocities_msg (Velocities): Estimated vehicle velocity data.
            yaw_rate_msg (Vector3Stamped): IMU acc with yaw_rate component.
            steering_angle_msg (SteeringAngle): Steering angle input from the vehicle.

        Returns:
            None
        """
        vx = velocities_msg.velocity_x
        vy = velocities_msg.velocity_y
        steering_angle = steering_angle_msg.steering_angle
        yaw_rate = yaw_rate_msg.vector.x

        L = 1.53  # wheelbase
        T = 1.2  # track width
        wf = 0.47  # front weight distribution

        slip_angle_beta = math.atan2(vy, vx)

        v_FL_x = vx - yaw_rate * (T / 2)
        v_FL_y = vy + yaw_rate * (L * wf)
        alpha_FL = abs(steering_angle - math.atan2(v_FL_y, v_FL_x))

        v_FR_x = vx + yaw_rate * (T / 2)
        v_FR_y = vy + yaw_rate * (L * wf)
        alpha_FR = abs(steering_angle - math.atan2(v_FR_y, v_FR_x))

        v_RL_x = vx - yaw_rate * (T / 2)
        v_RL_y = vy - yaw_rate * (L * (1 - wf))
        alpha_RL = abs(math.atan2(v_RL_y, v_RL_x))

        v_RR_x = vx + yaw_rate * (T / 2)
        v_RR_y = vy - yaw_rate * (L * (1 - wf))
        alpha_RR = abs(math.atan2(v_RR_y, v_RR_x))

        slip_angles_msg = Float64MultiArray()
        slip_angles_msg.data = [
            math.degrees(slip_angle_beta),
            math.degrees(alpha_FL),
            math.degrees(alpha_FR),
            math.degrees(alpha_RL),
            math.degrees(alpha_RR),
        ]

        self.slip_angle_pub.publish(slip_angles_msg)

        self.get_logger().info(
            f"Published Slip Angles (deg): CG: {math.degrees(slip_angle_beta):.2f}, FL: {math.degrees(alpha_FL):.2f}, FR: {math.degrees(alpha_FR):.2f}, RL: {math.degrees(alpha_RL):.2f}, RR: {math.degrees(alpha_RR):.2f}, Yaw Rate: {yaw_rate:.3f}"
        )

    def slip_ratio_callback(self, rl_rpm_msg, rr_rpm_msg, fl_rpm_msg, fr_rpm_msg):
        """
        Callback function for synchronized wheel speed sensor (WSS) data.
        Computes the left and right slip ratio using front and rear wheel velocities.

        Args:
            rl_rpm_msg (WheelRPM): Rear left wheel RPM message.
            rr_rpm_msg (WheelRPM): Rear right wheel RPM message.
            fl_rpm_msg (WheelRPM): Front left wheel RPM message.
            fr_rpm_msg (WheelRPM): Front right wheel RPM message.

        Returns:
            None
        """
        wheel_radius = 0.203

        V_RL = (rl_rpm_msg.rl_rpm * 2 * math.pi * wheel_radius) / 60
        V_RR = (rr_rpm_msg.rr_rpm * 2 * math.pi * wheel_radius) / 60
        V_FL = (fl_rpm_msg.fl_rpm * 2 * math.pi * wheel_radius) / 60
        V_FR = (fr_rpm_msg.fr_rpm * 2 * math.pi * wheel_radius) / 60

        avg_rear_velocity = 0.5 * (V_RR + V_RL)
        slip_ratio_left = (avg_rear_velocity / V_FL) - 1 if V_FL != 0 else float("nan")
        slip_ratio_right = (avg_rear_velocity / V_FR) - 1 if V_FR != 0 else float("nan")

        slip_ratios_msg = Float64MultiArray()
        slip_ratios_msg.data = [slip_ratio_left, slip_ratio_right]

        self.slip_ratio_pub.publish(slip_ratios_msg)

        self.get_logger().info(
            f"Published Slip Ratio: Left: {slip_ratio_left:.3f}, Right: {slip_ratio_right:.3f}"
        )


def main(args=None):
    """
    Main function for the Vehicle Dynamics module.
    Args:
        args (list): Optional command-line arguments.
    """
    rclpy.init(args=args)
    node = VehicleDynamicsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
