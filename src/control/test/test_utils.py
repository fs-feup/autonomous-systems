import unittest
import control.utils as utils
import numpy as np

class TestUtilsMethods(unittest.TestCase):
    def test_get_closest_point(self):
        points_array = np.asarray([[0, 2], [1, 0], [1, 1]])
        
        position = [0, 0]
        closest_point = utils.get_closest_point(position, points_array)
        self.assertEqual(list(closest_point), [1, 0])

        position = [-1, 1]
        closest_point = utils.get_closest_point(position, points_array)
        self.assertEqual(list(closest_point), [0, 2])

        points_array = np.asarray([[-4,10],[-3,10],[-1,10],[0,10],[1,10],[2,10],[2,10]])
        closest_point = utils.get_closest_point(position, points_array)
        self.assertEqual(list(closest_point), [-1, 10])
