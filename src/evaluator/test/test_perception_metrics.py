import unittest
import numpy as np
from evaluator.metrics import (
    get_average_difference,
    get_mean_squared_difference,
    get_inter_cones_distance,
    get_false_positives,
    get_duplicates
)


class TestEvaluatorMethods(unittest.TestCase):
    """
    Test case for the methods of the Evaluator class.
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
        Test case for the get_average_difference method when both arrays are non-empty.
        """
        average_difference = get_average_difference(self.output, self.ground_truth)
        self.assertAlmostEqual(
            average_difference, 0.7157378651666527, delta=0.00000000001
        )

    def test_get_average_difference2(self):
        """
        Test case for the get_average_difference method when the output array is empty.
        """
        average_difference = get_average_difference([], self.ground_truth)
        self.assertEqual(average_difference, float("inf"))

    def test_get_average_difference3(self):
        """
        Test case for the get_average_difference method when the ground truth array is empty.
        """
        with self.assertRaises(ValueError):
            get_average_difference(self.output, [])

    def test_get_mean_squared_error(self):
        """
        Test case for the get_mean_squared_difference method when the output array is non-empty.
        """
        mean_squared_error = get_mean_squared_difference(self.output, self.ground_truth)
        self.assertAlmostEqual(mean_squared_error, 1.03, delta=0.00001)

    def test_get_inter_cones_distance(self):
        """
        Test case for the get_inter_cones_distance method when the output array is non-empty.
        """
        inter_cones_distance = get_inter_cones_distance(self.output)
        self.assertAlmostEqual(inter_cones_distance, 1.6071067811865476, delta=0.005)

    def test_get_inter_cones_distance2(self):
        """
        Test case for the get_inter_cones_distance method when the output array is empty.
        """
        inter_cones_distance = get_inter_cones_distance(np.array([]))
        self.assertEqual(inter_cones_distance, 0)
    
    def test_get_false_positives(self):
        """
        Test case for the get_false_positives method.
        """
        threshold = 1.0
        false_positives = get_false_positives(self.output, self.ground_truth, threshold)
        self.assertEqual(false_positives, 1)

    def test_get_false_positives2(self):
        threshold = 1.0

        # Swapping for testing purposes
        output = self.ground_truth
        ground_truth = self.output
        false_positives = get_false_positives(output, ground_truth, threshold)
        self.assertEqual(false_positives, 5)

    def test_get_duplicates(self):
        threshold = 0.01
        count = get_duplicates(self.output,threshold)
        self.assertEqual(count, 0)

    def test_get_duplicates2(self):
        threshold = 1.0
        output = self.ground_truth
        count = get_duplicates(output, threshold)
        self.assertEqual(count, 1)


if __name__ == "__main__":
    unittest.main()
