from custom_interfaces.msg import ConeArray, VehicleState, PathPointArray
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from tf_transformations import euler_from_quaternion
from eufs_msgs.msg import ConeArrayWithCovariance, CarState
from nav_msgs.msg import Odometry
import numpy as np

def format_vehicle_state_msg(msg: VehicleState) -> tuple[np.ndarray, np.ndarray]:
    """!
    Formats the VehicleState message into a numpy array.

    Args:
        msg (VehicleState): Vehicle state message.

    Returns:
        np.ndarray: Numpy array of vehicle state.
    """

def format_cone_array_msg(msg: ConeArray) -> np.ndarray[np.ndarray]:
    """!
    Formats the ConeArray message into a numpy array.

    Args:
        msg (ConeArray): Cone array message.

    Returns:
        np.ndarray: Numpy array of arrays.
    """

def format_marker_array_msg(msg: MarkerArray) -> np.ndarray[np.ndarray]:
    """!
    Formats the MarkerArray message into a numpy array.

    Args:
        msg (MarkerArray): Marker array message.

    Returns:
        np.ndarray: Numpy array of arrays.
    """

def format_transform_stamped_msg(msg: TransformStamped) -> np.ndarray:
    """!
    Formats the TransformStamped message into a numpy array.

    Args:
        msg (TransformStamped): TransformStamped message.

    Returns:
        np.ndarray: Numpy array of transform.
    """

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

def format_car_state_msg(
    msg: CarState,
) -> tuple[np.ndarray, np.ndarray]:
    """!

    Formats the CarState message from eufs into a tuple of numpy arrays.

    Args: msg (CarState): CarState message from eufs

    Returns: tuple[np.ndarray, np.ndarray]: (state, velocities)
    """

def format_eufs_cone_array_with_covariance_msg(
    msg: ConeArrayWithCovariance,
) -> np.ndarray[np.ndarray]:
    """!

    Args:
        msg (ConeArrayWithCovariance): cone array message from eufs

    Returns:
        np.ndarray[np.ndarray]: [[x, y, color, confidence], ...]
    """

def format_nav_odometry_msg(msg: Odometry) -> tuple[np.ndarray, np.ndarray]:
    """!
    Formats the Odometry message into a numpy array.

    Args:
        msg (Odometry): Odometry message.

    Returns:
        np.ndarray: Numpy array of odometry.
    """

def format_path_point_array_msg(path_point_array: PathPointArray) -> np.ndarray:
    """!
    Converts a PathPointArray message into a numpy array.

    Args:
        path_point_array (PathPointArray): PathPointArray message.

    Returns:
        np.ndarray: Numpy array of path points.
    """

def format_point2d_msg(msg):
    """!
    Converts a Point2D message into a numpy array.

    Args:
        msg: Point2D message.

    Returns:
        np.ndarray: Numpy array of point.
    """