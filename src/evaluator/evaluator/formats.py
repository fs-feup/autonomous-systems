from custom_interfaces.msg import ConeArray, VehicleState, PathPointArray
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from tf_transformations import euler_from_quaternion
from eufs_msgs.msg import ConeArrayWithCovariance, CarState
from nav_msgs.msg import Odometry
import numpy as np

cone_color_dictionary: dict[str, int] = {
    "blue_cone": 0,
    "blue": 0,
    "yellow_cone": 1,
    "yellow": 1,
    "orange_cone": 2,
    "large_orange_cone": 3,
    "undefined": 4,
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


def format_car_state_msg(
    msg: CarState,
) -> tuple[np.ndarray, np.ndarray]:
    """!

    Formats the CarState message from eufs into a tuple of numpy arrays.

    Args: msg (CarState): CarState message from eufs

    Returns: tuple[np.ndarray, np.ndarray]: (state, velocities)
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
            ],
        ),
    )


def format_eufs_cone_array_with_covariance_msg(
    msg: ConeArrayWithCovariance,
):
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
    for i, cone_type in enumerate(cone_types):
        for cone in getattr(msg, cone_type):
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
    """_summary_

    Args:
        arr1 (np.ndarray): array in which each element's 2 initial values are x and y positions
        arr2 (np.ndarray): array in which each element's 2 initial values are x and y positions

    Returns:
        np.ndarray: array of elements from arr2 that are the closest to at least one element in arr1
    """
    closest_elements = []

    for point1 in arr1:
        closest_distance = float("inf")
        closest_element = None

        for point2 in arr2:
            distance = np.linalg.norm(point1[:2] - point2[:2])

            if distance < closest_distance:
                closest_distance = distance
                closest_element = point2

        if closest_element is not None and not any(
            np.array_equal(closest_element, elem) for elem in closest_elements
        ):
            closest_elements.append(closest_element)

    return np.array(closest_elements)


def get_blue_and_yellow_cones_after_msg_treatment(
    arr: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """!
    Divides the result of converting the EUFS or FSDS map message to np.ndarray into blue and yellow cones arrays, attributing orange
    cones into blue or yellow cones.

    Args:
        arr (np.ndarray): Array of cones with the following attributes: x, y, index, confidence, type.

    Returns:
        tuple[np.ndarray, np.ndarray]: the first array is the array of blue cones and the second contains yellow cones.
    """
    array1 = []
    array2 = []
    for element in arr:
        if element[2] == 0:
            array1.append(element)
        elif element[2] == 1:
            array2.append(element)
        elif element[2] in [2, 3]:
            closest_distance = float("inf")
            closest_array = None

            for array in [array1, array2]:
                for array_element in array:
                    distance = np.linalg.norm(element[:2] - array_element[:2])

                    if distance < closest_distance:
                        closest_distance = distance
                        closest_array = array

            if closest_array is not None:
                closest_array.append(element)
        else:
            raise ValueError("Invalid cone color index: %d" % element[2])

    return np.array(array1), np.array(array2)
