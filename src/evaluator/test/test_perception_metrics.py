import unittest
import numpy as np
from evaluator.evaluator import Evaluator

class TestEvaluatorMethods(unittest.TestCase):
    """
    Test case for the methods of the Evaluator class.
    """

    def setUp(self):
        """
        Set up the test environment by initializing output and ground truth arrays.
        """
        self.output = []
        self.output.append(np.array([1, 2, 0]))
        self.output.append(np.array([2, 1, 0]))
        self.output.append(np.array([2, 3.5, 0]))

        self.ground_truth = []
        self.ground_truth.append(np.array([1, 2, 0]))
        self.ground_truth.append(np.array([2.4, 0.8, 0]))
        self.ground_truth.append(np.array([4, 3, 0]))
        self.ground_truth.append(np.array([5, 5, 0]))
        self.ground_truth.append(np.array([3.5, 4.3, 0]))
        self.ground_truth.append(np.array([0, 0, 0]))
        self.ground_truth.append(np.array([4.3, 3.4, 0]))

    def test_get_average_difference1(self):
        """
        Test case for the get_average_difference method when both arrays are non-empty.
        """
        average_difference = Evaluator.get_average_difference(self.output, self.ground_truth)
        self.assertAlmostEqual(average_difference, 0.7157378651666527, delta=0.00000000001)

    def test_get_average_difference2(self):
        """
        Test case for the get_average_difference method when the output array is empty.
        """
        average_difference = Evaluator.get_average_difference([], self.ground_truth)
        self.assertEqual(average_difference, float('inf'))

    def test_get_average_difference3(self):
        """
        Test case for the get_average_difference method when the ground truth array is empty.
        """
        with self.assertRaises(ValueError):
            Evaluator.get_average_difference(self.output, [])

    def test_get_inter_cones_distance(self):
        """
        Test case for the get_inter_cones_distance method when the output array is non-empty.
        """
        inter_cones_distance = Evaluator.get_inter_cones_distance(self.output)
        self.assertAlmostEqual(inter_cones_distance, 1.6071067811865476, delta=0.005)
    
    def test_get_inter_cones_distance2(self):
        """
        Test case for the get_inter_cones_distance method when the output array is empty.
        """
        inter_cones_distance = Evaluator.get_inter_cones_distance([])
        self.assertEqual(inter_cones_distance, 0)

if __name__ == '__main__':
    unittest.main()
