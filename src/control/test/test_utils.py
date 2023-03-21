import unittest
import control.utils as utils
import numpy as np
import math

class TestUtilsMethods(unittest.TestCase):
    def test_get_closest_point(self):
        points_array = np.asarray([[0, 2], [1, 0], [1, 1]])
        
        position = [0, 0]
        closest_point, _ = utils.get_closest_point(position, points_array)
        self.assertEqual(list(closest_point), [1, 0])

        position = [-1, 1]
        closest_point, _ = utils.get_closest_point(position, points_array)
        self.assertEqual(list(closest_point), [0, 2])

        points_array = np.asarray([[-4,10],[-3,10],[-1,10],[0,10],[1,10],[2,10],[2,10]])
        closest_point, _ = utils.get_closest_point(position, points_array)
        self.assertEqual(list(closest_point), [-1, 10])

        points_array = np.array([[5,0], [4.9, 5.1]])
        pose_car = [-5, 0, math.radians(0)]
        closest_index = 0
        cte = utils.get_cte(closest_index, points_array, pose_car)
        self.assertEqual(round(cte, 0), 10)