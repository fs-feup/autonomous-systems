from custom_interfaces.msg import ConeArray, VehicleState, PathPointArray
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from tf_transformations import euler_from_quaternion
from eufs_msgs.msg import ConeArrayWithCovariance
from nav_msgs.msg import Odometry
import numpy as np

cone_color_dictionary: dict[str, int] = {
    "blue_cone": 0,
    "yellow_cone": 1,
    "orange_cone": 2,
    "large_orange_cone": 3,
    "unknown": 4,
}


def format_vehicle_state_msg(msg: VehicleState) -> tuple[np.ndarray, np.ndarray]:
    """!
    Formats the VehicleState message into a numpy array.

    Args:
        msg (VehicleState): Vehicle state message.

    Returns:
        np.ndarray: Numpy array of vehicle state.
    """
    return (
        np.array(
            [
                msg.position.x,
                msg.position.y,
                msg.theta,
            ]
        ),
        np.array([msg.linear_velocity, msg.linear_velocity, msg.angular_velocity]),
    )


def format_cone_array_msg(msg: ConeArray) -> np.ndarray[np.ndarray]:
    """!
    Formats the ConeArray message into a numpy array.

    Args:
        msg (ConeArray): Cone array message.

    Returns:
        np.ndarray: Numpy array of arrays.
    """
    output = []

    for cone in msg.cone_array:
        output.append(
            np.array(
                [
                    cone.position.x,
                    cone.position.y,
                    cone_color_dictionary[cone.color],
                    cone.confidence
                ]
            )
        )

    return np.array(output)


def format_marker_array_msg(msg: MarkerArray) -> np.ndarray[np.ndarray]:
    """!
    Formats the MarkerArray message into a numpy array.

    Args:
        msg (MarkerArray): Marker array message.

    Returns:
        np.ndarray: Numpy array of arrays.
    """
    output = []

    for marker in msg.markers:
        output.append(
            np.array([marker.pose.position.x, marker.pose.position.y, 0.0, 0.0])
        )

    return np.array(output)


def format_transform_stamped_msg(msg: TransformStamped) -> np.ndarray:
    """!
    Formats the TransformStamped message into a numpy array.

    Args:
        msg (TransformStamped): TransformStamped message.

    Returns:
        np.ndarray: Numpy array of transform.
    """
    yaw: float = euler_from_quaternion(
        [
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w,
        ]
    )[2]
    return np.array(
        [
            msg.transform.translation.x,
            msg.transform.translation.y,
            yaw,
        ]
    )


def format_twist_with_covariance_stamped_msg(
    msg: TwistWithCovarianceStamped,
) -> np.ndarray:
    """!
    Formats the TwistWithCovarianceStamped message into a numpy array.

    Args:
        msg (TwistWithCovarianceStamped): TwistWithCovarianceStamped message.

    Returns:
        np.ndarray: Numpy array of twist (used for velocities).
    """
    return np.array(
        [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]
    )


def format_eufs_cone_array_with_covariance_msg(
    msg: ConeArrayWithCovariance,
) -> np.ndarray[np.ndarray]:
    """!

    Args:
        msg (ConeArrayWithCovariance): cone array message from eufs

    Returns:
        np.ndarray[np.ndarray]: [[x, y, color, confidence], ...]
    """
    output: list = []
    cone_types: list[str] = [
        "blue_cones",
        "yellow_cones",
        "orange_cones",
        "big_orange_cones",
        "unknown_color_cones",
    ]
    for cone_type in cone_types:
        for i, cone in enumerate(getattr(msg, cone_type)):
            output.append(
                np.array(
                    [
                        cone.point.x,
                        cone.point.y,
                        i,
                        1.0,
                    ]  # TODO: confidence dependent on the cone's covariance
                )
            )

    return np.array(output)


def format_nav_odometry_msg(msg: Odometry) -> np.ndarray:
    """!
    Formats the Odometry message into a numpy array.

    Args:
        msg (Odometry): Odometry message.

    Returns:
        np.ndarray: Numpy array of odometry.
    """
    return np.array(
        [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            euler_from_quaternion(
                [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ]
            )[2],
        ]
    )


def format_path_point_array_msg(path_point_array: PathPointArray):
    """!
    Converts a PathPointArray message into a numpy array.

    Args:
        path_point_array (PathPointArray): PathPointArray message.

    Returns:
        np.ndarray: Numpy array of path points.
    """
    path_list = []

    for point in path_point_array:
        path_list.append(np.array([point.x, point.y, 0.0]))

    return path_list


def format_point2d_msg(msg):
    """!
    Converts a Point2D message into a numpy array.

    Args:
        msg: Point2D message.

    Returns:
        np.ndarray: Numpy array of point.
    """
    return np.array([msg.x, msg.y])