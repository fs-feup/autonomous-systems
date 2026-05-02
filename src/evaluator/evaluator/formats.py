from custom_interfaces.msg import (
    ConeArray,
    VehicleState,
    PathPointArray,
    Pose,
    Velocities,
)
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import numpy as np

cone_color_dictionary: dict[str, int] = {
    "blue_cone": 0,
    "blue": 0,
    "yellow_cone": 1,
    "yellow": 1,
    "orange_cone": 2,
    "large_orange_cone": 3,
    "unknown": 4,
}


def format_vehicle_pose_msg(msg: Pose) -> tuple[np.ndarray, np.ndarray]:
    """!
    Formats the Pose message into a numpy array.

    Args:
        msg (Pose): Vehicle pose message.

    Returns:
        np.ndarray: Numpy array of vehicle pose.
    """
    return np.array(
        [
            msg.x,
            msg.y,
            msg.theta,
        ]
    )


def format_velocities_msg(msg: Velocities) -> tuple[np.ndarray, np.ndarray]:
    """!
    Formats the Velocities message into a numpy array.

    Args:
        msg (Velocities): Vehicle velocities message.

    Returns:
        np.ndarray: Numpy array of vehicle velocities.
    """
    return np.array(
        [
            msg.velocity_x,
            msg.velocity_y,
            msg.angular_velocity,
        ]
    )


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


def format_cone_array_msg(msg: ConeArray):
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
                    cone.confidence,
                ]
            )
        )

    return np.array(output)


def get_color_number_from_rgb(r, g, b):
    if b == 1:
        return 0
    elif r == 1 and g == 1:
        return 1
    else:
        return 2


def format_marker_array_msg(msg: MarkerArray):
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
            np.array(
                [
                    marker.pose.position.x,
                    marker.pose.position.y,
                    get_color_number_from_rgb(
                        marker.color.r, marker.color.g, marker.color.b
                    ),
                    0.0,
                ]
            )
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


def format_nav_odometry_msg(msg: Odometry) -> tuple[np.ndarray, np.ndarray]:
    """!
    Formats the Odometry message into a numpy array.

    Args:
        msg (Odometry): Odometry message.

    Returns:
        np.ndarray: Numpy array of odometry.
    """
    return (
        np.array(
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
        ),
        np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.angular.z,
            ]
        ),
    )


def format_path_point_array_msg(path_point_array: PathPointArray) -> np.ndarray:
    """!
    Converts a PathPointArray message into a numpy array.

    Args:
        path_point_array (PathPointArray): PathPointArray message.

    Returns:
        np.ndarray: Numpy array of path points.
    """
    path_list = []

    for path_point in path_point_array.pathpoint_array:
        path_list.append(
            np.array(
                [
                    path_point.x,
                    path_point.y,
                    path_point.v,
                ]
            )
        )

    return np.array(path_list)


def format_point2d_msg(msg):
    """!
    Converts a Point2D message into a numpy array.

    Args:
        msg: Point2D message.

    Returns:
        np.ndarray: Numpy array of point.
    """
    return np.array([msg.x, msg.y])


def find_closest_elements(arr1: np.ndarray, arr2: np.ndarray) -> np.ndarray:
    """Find the closest elements in arr2 for each element in arr1.

    Args:
        arr1 (np.ndarray): array in which each element's 2 initial values are x and y positions
        arr2 (np.ndarray): array in which each element's 2 initial values are x and y positions

    Returns:
        np.ndarray: array of elements from arr2 that are the closest to at least one element in arr1
    """
    # Extract the x and y positions
    arr1_xy = arr1[:, :2]
    arr2_xy = arr2[:, :2]

    # Calculate the squared Euclidean distances
    distances = np.linalg.norm(
        arr1_xy[:, np.newaxis, :] - arr2_xy[np.newaxis, :, :], axis=2
    )

    # Find the indices of the closest elements in arr2 for each element in arr1
    closest_indices = np.argmin(distances, axis=1)

    # Get the unique closest elements from arr2
    closest_elements = np.unique(arr2[closest_indices], axis=0)

    return closest_elements
