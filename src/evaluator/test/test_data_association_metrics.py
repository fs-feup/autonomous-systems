import unittest
import numpy.testing as npt
import numpy as np
from evaluator.metrics import (
    get_average_difference,
    get_mean_squared_difference,
    get_inter_cones_distance,
    get_false_positives,
    build_adjacency_matrix,
    get_duplicates,
)


class TestDataAssociationMetrics(unittest.TestCase):
    """
    Test case for the data association methods of the Evaluator class.
    """

    def setUp(self):
        """
        Set up the test environment by initializing output and ground truth arrays.
        """
        self.output = np.array([
            [1, 2, 0],
            [2, 1, 0],
            [2, 3.5, 0]
        ])

        self.ground_truth = np.array([
            np.array([1, 2, 0]),
            np.array([2.4, 0.8, 0]),
            np.array([4, 3, 0]),
            np.array([5, 5, 0]),
            np.array([3.5, 4.3, 0]),
            np.array([0, 0, 0]),
            np.array([4.3, 3.4, 0])
        ])

    def test_get_average_difference1(self):
        """
        Test the get_average_difference function when both arrays are non-empty.
        Expects a correct average difference based on manually calculated values.
        """
        average_difference = get_average_difference(self.output, self.ground_truth)
        self.assertAlmostEqual(average_difference, 0.7157378651666527, delta=1e-10)

    def test_get_average_difference2(self):
        """
        Test the get_average_difference function when the output array is empty.
        Expects infinity as the return value.
        """
        average_difference = get_average_difference([], self.ground_truth)
        self.assertEqual(average_difference, float("inf"))

    def test_get_average_difference3(self):
        """
        Test the get_average_difference function when the ground truth array is empty.
        Expects a ValueError due to incompatible data.
        """
        with self.assertRaises(ValueError):
            get_average_difference(self.output, [])

    def test_get_mean_squared_error(self):
        """
        Test the get_mean_squared_difference function with valid output and ground truth arrays.
        Expects a mean squared difference close to the manually calculated value.
        """
        mean_squared_error = get_mean_squared_difference(self.output, self.ground_truth)
        self.assertAlmostEqual(mean_squared_error, 1.03, delta=0.00001)

    def test_get_inter_cones_distance(self):
        """
        Test the get_inter_cones_distance function for a non-empty output array.
        Expects a correct average inter-cone distance based on the output array.
        """
        inter_cones_distance = get_inter_cones_distance(self.output)
        self.assertAlmostEqual(inter_cones_distance, 1.6071067811865476, delta=0.005)

    def test_get_inter_cones_distance2(self):
        """
        Test the get_inter_cones_distance function for an empty output array.
        Expects a zero return value since there are no cones.
        """
        inter_cones_distance = get_inter_cones_distance(np.array([]))
        self.assertEqual(inter_cones_distance, 0)
    
    def test_get_false_positives(self):
        """
        Test the get_false_positives function with a threshold of 1.0.
        Expects the number of false positives based on the given threshold and arrays.
        """
        threshold = 1.0
        false_positives = get_false_positives(self.output, self.ground_truth, threshold)
        self.assertEqual(false_positives, 1)

    def test_get_false_positives2(self):
        """
        Test the get_false_positives function with arrays swapped and a threshold of 1.0.
        Expects a different number of false positives when the arrays are swapped.
        """
        threshold = 1.0
        output = self.ground_truth
        ground_truth = self.output
        false_positives = get_false_positives(output, ground_truth, threshold)
        self.assertEqual(false_positives, 5)

    def test_get_duplicates(self):
        """
        Test the get_duplicates function with a threshold of 0.01.
        Expects no duplicates since the output array does not contain any within this threshold.
        """
        threshold = 0.01
        count = get_duplicates(self.output, threshold)
        self.assertEqual(count, 0)

    def test_get_duplicates2(self):
        """
        Test the get_duplicates function with a threshold of 1.0.
        Expects 1 duplicate since the ground truth array contains a close match within this threshold.
        """
        threshold = 1.0
        output = self.ground_truth
        count = get_duplicates(output, threshold)
        self.assertEqual(count, 1)

    def test_adjacency_matrix(self):
        """
        Test the build_adjacency_matrix function with a subarray of the ground truth array.
        Expects an adjacency matrix calculated based on pairwise distances.
        """
        subarray = self.ground_truth[:2, :]
        adjacency_matrix = build_adjacency_matrix(subarray)
        expected = np.array([
            np.array([0, 1.8439088914585773]),  # Manually computed distance
            np.array([1.8439088914585773, 0])
        ])
        npt.assert_array_almost_equal(adjacency_matrix, expected, decimal=6)

    def test_adjacency_matrix2(self):
        """
        Test the build_adjacency_matrix function with a different subarray of the ground truth array.
        Expects an adjacency matrix calculated for a different subset of cones.
        """
        subarray = self.ground_truth[1:3, :]
        adjacency_matrix = build_adjacency_matrix(subarray)
        expected = np.array([
            np.array([0, 2.720294101747089]),  # Manually computed distance
            np.array([2.720294101747089, 0])
        ])
        npt.assert_array_almost_equal(adjacency_matrix, expected, decimal=6)
