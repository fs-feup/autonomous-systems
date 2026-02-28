import math
import sys
from select import select
import termios
import tty

import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Velocities
from geometry_msgs.msg import Vector3Stamped


def get_key(input_stream, settings, timeout):
    tty.setraw(input_stream.fileno())
    rlist, _, _ = select([input_stream], [], [], timeout)
    key = input_stream.read(1) if rlist else ""
    termios.tcsetattr(input_stream, termios.TCSADRAIN, settings)
    return key


def rotation_matrix_from_euler(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def mat_transpose(matrix):
    return [
        [matrix[0][0], matrix[1][0], matrix[2][0]],
        [matrix[0][1], matrix[1][1], matrix[2][1]],
        [matrix[0][2], matrix[1][2], matrix[2][2]],
    ]


def mat_mul(left, right):
    return [
        [
            left[0][0] * right[0][0]
            + left[0][1] * right[1][0]
            + left[0][2] * right[2][0],
            left[0][0] * right[0][1]
            + left[0][1] * right[1][1]
            + left[0][2] * right[2][1],
            left[0][0] * right[0][2]
            + left[0][1] * right[1][2]
            + left[0][2] * right[2][2],
        ],
        [
            left[1][0] * right[0][0]
            + left[1][1] * right[1][0]
            + left[1][2] * right[2][0],
            left[1][0] * right[0][1]
            + left[1][1] * right[1][1]
            + left[1][2] * right[2][1],
            left[1][0] * right[0][2]
            + left[1][1] * right[1][2]
            + left[1][2] * right[2][2],
        ],
        [
            left[2][0] * right[0][0]
            + left[2][1] * right[1][0]
            + left[2][2] * right[2][0],
            left[2][0] * right[0][1]
            + left[2][1] * right[1][1]
            + left[2][2] * right[2][1],
            left[2][0] * right[0][2]
            + left[2][1] * right[1][2]
            + left[2][2] * right[2][2],
        ],
    ]


def mat_vec_mul(matrix, vector):
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def gravity_from_euler(roll, pitch, gravity):
    sin_roll = math.sin(roll)
    cos_roll = math.cos(roll)
    sin_pitch = math.sin(pitch)
    cos_pitch = math.cos(pitch)

    return (
        -gravity * sin_pitch,
        gravity * sin_roll * cos_pitch,
        gravity * cos_roll * cos_pitch,
    )


class Integrator(Node):
    def __init__(self):
        super().__init__("integrator")
        self.declare_parameter("imu_acceleration_topic", "/imu/acceleration")
        self.declare_parameter("imu_euler_topic", "/filter/euler")
        self.declare_parameter("velocity_topic", "/imu/integrated_velocity")
        self.declare_parameter("calibration_samples", 1000)
        self.declare_parameter("gravity_magnitude", 9.80920)
        self.declare_parameter("euler_in_degrees", True)
        self.declare_parameter("output_in_body_frame", True)
        self.declare_parameter("output_frame_id", "imu_corrected")

        imu_topic = self.get_parameter("imu_acceleration_topic").get_parameter_value().string_value
        euler_topic = self.get_parameter("imu_euler_topic").get_parameter_value().string_value
        velocity_topic = self.get_parameter("velocity_topic").get_parameter_value().string_value

        self.calibration_samples = max(
            1, int(self.get_parameter("calibration_samples").get_parameter_value().integer_value)
        )
        self.gravity_magnitude = None
        self.euler_in_degrees = bool(
            self.get_parameter("euler_in_degrees").get_parameter_value().bool_value
        )
        self.output_in_body_frame = bool(
            self.get_parameter("output_in_body_frame").get_parameter_value().bool_value
        )
        self.output_frame_id = (
            self.get_parameter("output_frame_id").get_parameter_value().string_value
        )

        self.velocity_publisher = self.create_publisher(Velocities, velocity_topic, 10)
        self.imu_subscription = self.create_subscription(
            Vector3Stamped,
            imu_topic,
            self.imu_callback,
            10,
        )
        self.euler_subscription = self.create_subscription(
            Vector3Stamped,
            euler_topic,
            self.euler_callback,
            10,
        )

        self.velocity_world = [0.0, 0.0, 0.0]
        self.last_time = None

        self.latest_euler = None
        self._warned_no_orientation = False

        self._calibration_count = 0
        self._roll_sin_sum = 0.0
        self._roll_cos_sum = 0.0
        self._pitch_sin_sum = 0.0
        self._pitch_cos_sum = 0.0
        self._yaw_sin_sum = 0.0
        self._yaw_cos_sum = 0.0
        self._accel_sum = [0.0, 0.0, 0.0]
        self._g_unit_sum = [0.0, 0.0, 0.0]
        self._gravity_norm_sum = 0.0

        self.initial_rotation = None
        self.accel_bias = None
        self.estimated_gravity = None

        self.get_logger().info(
            "Integrator started. IMU topic: "
            f"{imu_topic} | Euler topic: {euler_topic} | velocity topic: {velocity_topic}"
        )
        self.get_logger().info(
            "Calibration samples: "
            f"{self.calibration_samples} | gravity estimated from IMU data"
        )
        self.get_logger().info(
            "Velocity output frame: "
            f"{'body' if self.output_in_body_frame else 'reference'}"
        )

    def euler_callback(self, msg: Vector3Stamped):
        roll = msg.vector.x
        pitch = msg.vector.y
        yaw = msg.vector.z
        if self.euler_in_degrees:
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)
        self.latest_euler = (roll, pitch, yaw)

    def _accumulate_calibration(self, acceleration, euler):
        roll, pitch, yaw = euler
        accel_norm = math.sqrt(
            acceleration[0] ** 2 + acceleration[1] ** 2 + acceleration[2] ** 2
        )
        g_unit = gravity_from_euler(roll, pitch, 1.0)
        self._gravity_norm_sum += accel_norm
        self._accel_sum[0] += acceleration[0]
        self._accel_sum[1] += acceleration[1]
        self._accel_sum[2] += acceleration[2]
        self._g_unit_sum[0] += g_unit[0]
        self._g_unit_sum[1] += g_unit[1]
        self._g_unit_sum[2] += g_unit[2]

        self._roll_sin_sum += math.sin(roll)
        self._roll_cos_sum += math.cos(roll)
        self._pitch_sin_sum += math.sin(pitch)
        self._pitch_cos_sum += math.cos(pitch)
        self._yaw_sin_sum += math.sin(yaw)
        self._yaw_cos_sum += math.cos(yaw)

        self._calibration_count += 1
        if self._calibration_count < self.calibration_samples:
            return False

        roll_avg = math.atan2(self._roll_sin_sum, self._roll_cos_sum)
        pitch_avg = math.atan2(self._pitch_sin_sum, self._pitch_cos_sum)
        yaw_avg = math.atan2(self._yaw_sin_sum, self._yaw_cos_sum)
        self.initial_rotation = rotation_matrix_from_euler(roll_avg, pitch_avg, yaw_avg)

        self.estimated_gravity = self._gravity_norm_sum / self._calibration_count
        self.gravity_magnitude = self.estimated_gravity
        accel_avg = [value / self._calibration_count for value in self._accel_sum]
        g_unit_avg = [value / self._calibration_count for value in self._g_unit_sum]
        self.accel_bias = [
            accel_avg[0] - self.gravity_magnitude * g_unit_avg[0],
            accel_avg[1] - self.gravity_magnitude * g_unit_avg[1],
            accel_avg[2] - self.gravity_magnitude * g_unit_avg[2],
        ]

        self.get_logger().info(
            "IMU calibration complete. Estimated gravity: "
            f"{self.estimated_gravity:.5f} m/s^2 | Accel bias: "
            f"[{self.accel_bias[0]:.3f}, {self.accel_bias[1]:.3f}, "
            f"{self.accel_bias[2]:.3f}]"
        )
        return True

    def imu_callback(self, msg: Vector3Stamped):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if current_time == 0.0:
            current_time = self.get_clock().now().nanoseconds * 1e-9

        if self.latest_euler is None:
            if not self._warned_no_orientation:
                self.get_logger().warn(
                    "No IMU Euler data received yet; waiting before integrating."
                )
                self._warned_no_orientation = True
            self.last_time = current_time
            return

        acceleration = (msg.vector.x, msg.vector.y, msg.vector.z)
        if self.accel_bias is None:
            calibrated = self._accumulate_calibration(acceleration, self.latest_euler)
            if calibrated:
                self.last_time = current_time
            return

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        if dt <= 0.0:
            self.get_logger().warn("Received non-positive dt from IMU message; skipping update.")
            self.last_time = current_time
            return

        roll, pitch, yaw = self.latest_euler
        rotation = rotation_matrix_from_euler(roll, pitch, yaw)
        gravity_imu = gravity_from_euler(roll, pitch, self.gravity_magnitude)
        linear_acc = (
            acceleration[0] - gravity_imu[0] - self.accel_bias[0],
            acceleration[1] - gravity_imu[1] - self.accel_bias[1],
            acceleration[2] - gravity_imu[2] - self.accel_bias[2],
        )

        if self.initial_rotation is not None:
            rotation_rel = mat_mul(mat_transpose(self.initial_rotation), rotation)
        else:
            rotation_rel = rotation

        linear_acc_world = mat_vec_mul(rotation_rel, linear_acc)

        self.velocity_world[0] += linear_acc_world[0] * dt
        self.velocity_world[1] += linear_acc_world[1] * dt
        self.velocity_world[2] += linear_acc_world[2] * dt
        self.last_time = current_time

        self.publish_velocity(msg.header.stamp, msg.header.frame_id, rotation_rel)

    def publish_velocity(self, stamp, frame_id, rotation_rel=None):
        velocity_msg = Velocities()
        velocity_msg.header.stamp = stamp
        velocity_msg.header.frame_id = self.output_frame_id or frame_id
        if self.output_in_body_frame and rotation_rel is not None:
            velocity_body = mat_vec_mul(mat_transpose(rotation_rel), self.velocity_world)
        else:
            velocity_body = self.velocity_world
        velocity_out = (-velocity_body[1], velocity_body[0], velocity_body[2])
        velocity_msg.velocity_x = float(velocity_out[0])
        velocity_msg.velocity_y = float(velocity_out[1])
        velocity_msg.angular_velocity = 0.0
        velocity_msg.covariance = [0.0] * 9
        self.velocity_publisher.publish(velocity_msg)

    def reset(self):
        self.velocity_world = [0.0, 0.0, 0.0]
        self.last_time = None
        now = self.get_clock().now().to_msg()
        self.publish_velocity(now, "", None)
        self.get_logger().info("Integrator reset: velocity set to 0 and count reset.")


def main(args=None):
    rclpy.init(args=args)
    node = Integrator()

    input_stream = sys.stdin
    input_stream_owned = False
    settings = None
    if not input_stream.isatty():
        try:
            input_stream = open("/dev/tty")
            input_stream_owned = True
            node.get_logger().info("Using /dev/tty for keyboard reset input.")
        except OSError:
            input_stream = None

    if input_stream is not None and input_stream.isatty():
        settings = termios.tcgetattr(input_stream)
    else:
        node.get_logger().warn("No TTY detected; keyboard reset (R) is disabled.")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if settings is None:
                continue

            key = get_key(input_stream, settings, 0.0)
            if key in ("r", "R"):
                node.reset()
            elif key == "q" or key == "\x03":
                break
    finally:
        if settings is not None and input_stream is not None:
            termios.tcsetattr(input_stream, termios.TCSADRAIN, settings)
        if input_stream_owned and input_stream is not None:
            input_stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
