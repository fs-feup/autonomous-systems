import unittest
from driverless.src.control.control.pid_utils import get_cte, get_closest_point
import numpy as np
import math

class TestUtilsMethods(unittest.TestCase):
    def test_get_closest_point(self):
        points_array = np.asarray([[0, 2], [1, 0], [1, 1]])
        
        position = [0, 0]
        closest_point, _ = get_closest_point(position, points_array)
        self.assertEqual(list(closest_point), [1, 0])

        position = [-1, 1]
        closest_point, _ = get_closest_point(position, points_array)
        self.assertEqual(list(closest_point), [0, 2])

        points_array = np.asarray([[-4,10],[-3,10],[-1,10],[0,10],[1,10],[2,10],[2,10]])
        closest_point, _ = get_closest_point(position, points_array)
        self.assertEqual(list(closest_point), [-1, 10])


    def test_get_cte(self):
        test_flag = True

        for same_dir in [True, False]:
            for deg_angle in range(-180, 180 + 15, 15):
                angle = math.radians(deg_angle + 90)

                x_car = math.cos(angle)
                y_car = math.sin(angle)

                # 70 can be any number. Just to avoid inf divisions
                yaw_car = angle + math.pi/2 + 70

                x_track_1 = math.cos(angle + math.pi)
                y_track_1 = math.sin(angle + + math.pi)
                yaw_track = angle + math.pi/2 if same_dir else angle - math.pi/2

                x_track_2 = x_track_1 + 0.1*math.cos(yaw_track)
                y_track_2 = y_track_1 + 0.1*math.sin(yaw_track)
                
                points_array = np.array([[x_track_1, y_track_1],
                                         [x_track_2, y_track_2]])
                pose_car = [x_car, y_car, yaw_car]
                closest_index = 0
                cte = get_cte(closest_index, points_array, pose_car)

                ans_mult = 1 if same_dir else -1

                if round(cte, 3) != ans_mult*2.0:
                    test_flag = False
            
        self.assertEqual(test_flag, True)
