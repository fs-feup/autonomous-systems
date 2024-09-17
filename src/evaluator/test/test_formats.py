import unittest
import numpy.testing as npt
import numpy as np
from custom_interfaces.msg import ConeArray, VehicleState, PathPointArray, Cone, PathPoint, Point2d
from eufs_msgs.msg import ConeArrayWithCovariance, CarState, ConeWithCovariance
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from evaluator.formats import (
    format_vehicle_state_msg,
    format_cone_array_msg,
    format_marker_array_msg,
    format_transform_stamped_msg,
    format_twist_with_covariance_stamped_msg,
    format_car_state_msg,
    format_eufs_cone_array_with_covariance_msg,
    format_nav_odometry_msg,
    format_path_point_array_msg,
    format_point2d_msg
)


class TestFormats(unittest.TestCase):
    """
    Test case for the format methods of the Evaluator class.
    """

    def setUp(self):
        """
        Set up the test environment by initializing expected value
        """

        self.expected_position = np.array(
            [
                7.0,
                3.0,
                42.0,
            ]
        ),

        self.expected_velocities = np.array(
            [
                1.0, 
                1.0, 
                2.0
            ]
        ),
    
        self.expected_cone_array = np.array(
            [
                np.array(
                [
                    1,
                    2,
                    0,
                    73,
                ]
            )
            ]
            
        )

        self.expected_stamped_message = np.array(
            [
                1,
                2,
                1.5708
            ]
        )

        self.expected_path_point_array = np.array(
            [
                np.array(
                    [
                        6,
                        9,
                        3,
                    ]
                )
            ]
        )


    def test_format_vehicle_state_msg(self):
        """
        Test case to check the formatting of a VehicleState message
        """

        msg = VehicleState()
        msg.position.x = 7.0
        msg.position.y = 3.0
        msg.theta = 42.0
        msg.linear_velocity = 1.0
        msg.angular_velocity = 2.0

        format_position, format_velocities = format_vehicle_state_msg(msg)

        npt.assert_array_almost_equal(format_position, self.expected_position[0], decimal=6)
        npt.assert_array_almost_equal(format_velocities, self.expected_velocities[0], decimal=6)

    def test_format_cone_array_msg(self):
        """
        Test case to check the formatting of a ConeArray message
        """

        cone_msg = Cone()
        cone_msg.position.x = 1.0
        cone_msg.position.y = 2.0
        cone_msg.color = 'blue'
        cone_msg.confidence = 73.0
        msg = ConeArray()
        msg.cone_array.append(cone_msg)

        formated_position = format_cone_array_msg(msg)

        npt.assert_array_almost_equal(formated_position, self.expected_cone_array, decimal=6)

    def test_format_marker_array(self):
        """
        Test case to check the formatting of a MarkerArray message
        """

        marker = Marker()
        marker.pose.position.x = 1.0
        marker.pose.position.y = 2.0
        msg = MarkerArray()
        msg.markers.append(marker)

        formated_msg = format_marker_array_msg(msg)
        formated_msg[0][3] = 73

        npt.assert_array_almost_equal(formated_msg, self.expected_cone_array, decimal=6)

    def test_format_transform_stamped(self):
        """
        Test case to check the formatting of a TransformStamped message
        """

        msg = TransformStamped()
        msg.transform.translation.x = 1.0
        msg.transform.translation.y = 2.0 
        msg.transform.translation.z = 0.0 
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = 0.7071
        msg.transform.rotation.w = 0.7071

        formated_msg = format_transform_stamped_msg(msg)

        npt.assert_array_almost_equal(formated_msg, self.expected_stamped_message, decimal=4)


    def test_twist_with_covariance_stamped(self):
        """
        Test case to check the formatting of a TwistWithCovarianceStamped message
        """

        msg = TwistWithCovarianceStamped()
        msg.twist.twist.linear.x = 1.0
        msg.twist.twist.linear.y = 1.0
        msg.twist.twist.angular.z = 2.0

        formated_msg = format_twist_with_covariance_stamped_msg(msg)

        npt.assert_array_almost_equal(formated_msg, self.expected_velocities[0], decimal=6)

    def test_format_car_state(self):
        """
        Test case to check the formatting of a CarState message
        """

        msg = CarState()
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 2.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.7071
        msg.pose.pose.orientation.w = 0.7071
        msg.twist.twist.linear.x = 1.0
        msg.twist.twist.linear.y = 1.0
        msg.twist.twist.angular.z = 2.0

        formated_msg1, formated_msg2 = format_car_state_msg(msg)

        npt.assert_array_almost_equal(formated_msg1, self.expected_stamped_message, decimal=4)
        npt.assert_array_almost_equal(formated_msg2, self.expected_velocities[0], decimal=4)

    def test_format_eufs_cone_array_with_covariance(self):
        """
        Test case to check the formatting of a ConeArrayWithCovariance message
        """

        msg = ConeArrayWithCovariance()
        cone = ConeWithCovariance()
        cone.point.x = 1.0
        cone.point.y = 2.0
        msg.blue_cones.append(cone)

        formated_msg = format_eufs_cone_array_with_covariance_msg(msg)
        formated_msg[0][3] = 73

        npt.assert_array_almost_equal(formated_msg, self.expected_cone_array, decimal=4)

    def test_format_nav_odometry(self):
        """
        Test case to check the formatting of a Odometry message
        """

        msg = Odometry()
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 2.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.7071
        msg.pose.pose.orientation.w = 0.7071
        msg.twist.twist.linear.x = 1.0
        msg.twist.twist.linear.y = 1.0
        msg.twist.twist.angular.z = 2.0

        formated_msg1, formated_msg2 = format_nav_odometry_msg(msg)

        npt.assert_array_almost_equal(formated_msg1, self.expected_stamped_message, decimal=4)
        npt.assert_array_almost_equal(formated_msg2, self.expected_velocities[0], decimal=4)

    def test_format_path_point_array(self):
        """
        Test case to check the formatting of a PathPointArray message
        """

        msg = PathPointArray()
        path_point = PathPoint()
        path_point.x = 6.0
        path_point.y = 9.0
        path_point.v = 3.0
        msg.pathpoint_array.append(path_point)

        formated_msg = format_path_point_array_msg(msg)

        npt.assert_array_almost_equal(formated_msg, self.expected_path_point_array, decimal=6)

    def test_format_point2d(self):
        """
        Test case to check the formatting of a Point2d message
        """

        msg = Point2d()
        msg.x = 2.0
        msg.y = 1.0

        formated_msg = format_point2d_msg(msg)

        npt.assert_array_almost_equal(formated_msg, [2, 1], decimal=6)
