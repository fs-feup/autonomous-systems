#from control.pid_utils import get_cte, get_closest_point
from control.control_node import ControlNode
from custom_interfaces.msg import PointArray, Point2d
import rclpy
import unittest
#import numpy as np
#import math
import datetime
import csv
from control.config import Params

P = Params()

class TestUtilsMethods(unittest.TestCase):
    # def test_get_closest_point(self):
    #     points_array = np.asarray([[0, 2], [1, 0], [1, 1]])
        
    #     position = [0, 0]
    #     closest_point, _ = get_closest_point(position, points_array)
    #     self.assertEqual(list(closest_point), [1, 0])

    #     position = [-1, 1]
    #     closest_point, _ = get_closest_point(position, points_array)
    #     self.assertEqual(list(closest_point), [0, 2])

    #     points_array = np.asarray([[-4,10],[-3,10],\
    #       [-1,10],[0,10],[1,10],[2,10],[2,10]])
    #     closest_point, _ = get_closest_point(position, points_array)
    #     self.assertEqual(list(closest_point), [-1, 10])


    # def test_get_cte(self):
    #     test_flag = True

    #     for same_dir in [True, False]:
    #         for deg_angle in range(-180, 180 + 15, 15):
    #             angle = math.radians(deg_angle + 90)

    #             x_car = math.cos(angle)
    #             y_car = math.sin(angle)

    #             # 70 can be any number. Just to avoid inf divisions
    #             yaw_car = angle + math.pi/2 + 70

    #             x_track_1 = math.cos(angle + math.pi)
    #             y_track_1 = math.sin(angle + + math.pi)
    #             yaw_track = angle + math.pi/2 if same_dir else angle - math.pi/2

    #             x_track_2 = x_track_1 + 0.1*math.cos(yaw_track)
    #             y_track_2 = y_track_1 + 0.1*math.sin(yaw_track)
                
    #             points_array = np.array([[x_track_1, y_track_1],
    #                                      [x_track_1, y_track_1],
    #                                      [x_track_2, y_track_2],
    #                                      [x_track_2, y_track_2]])
    #             pose_car = [x_car, y_car, yaw_car]
    #             closest_index = 0

    #             print(points_array)

    #             cte = get_cte(closest_index, points_array, pose_car)

    #             ans_mult = 1 if same_dir else -1

    #             if round(cte, 3) != ans_mult*2.0:
    #                 test_flag = False
            
    #     self.assertEqual(test_flag, True)

    def test_pid_callback(self):
        dtsum = 0
        dt2sum = 0
        dt3sum = 0
        firstdt = 0
        firstdt2 = 0
        firstdt3 = 0
        no_iters = 100

        position = Point2d()
        position.x = 0.0
        position.y = -1.0
        yaw = 0

        raw_path = [(1.5,0.0), (1.75,1.0), (2.0,2.0), (2.5,2.4), (3.0,2.8),\
            (3.5,3.15), (4.0,3.5), (4.25,4.25), (4.5,5.0), (4.7,5.5), (5.0, 6.0)]

        for i in range(no_iters):
            t0 = datetime.datetime.now() 
            rclpy.init()
            t1 = datetime.datetime.now()   
            control_node = ControlNode()

            recv_path = PointArray()
            for point_tuple in raw_path:
                point = Point2d()
                point.x = point_tuple[0]
                point.y = point_tuple[1]
                recv_path.points.append(point)

            control_node.path_callback(recv_path)
            t2 = datetime.datetime.now() 

            control_node.pid_callback(position, yaw)

            t3 = datetime.datetime.now()       
            dt = t3 - t0
            dt2 = t2 - t1
            dt3 = t3 - t2

            dtsum += dt.microseconds
            dt2sum += dt2.microseconds
            dt3sum += dt3.microseconds

            rclpy.shutdown()

            if i == 0:
                firstdt = dtsum * 1e-3
                firstdt2 = dt2sum * 1e-3
                firstdt3 = dt2sum * 1e-3
                print("First Node and Pid calculated in ", firstdt, " ms")
                print("First Pid path processing calculated in ", firstdt2, " ms")
                print("First Pid only calculated in ", firstdt3, " ms")
                
        f = open('control/test/control_measures.csv', 'a')
        writer = csv.writer(f, delimiter=',')
        writer.writerow(['control', 'pid', 'path_processing-' +\
             str(len(raw_path)) + 'pts', dt2sum /  no_iters * 1e-3, firstdt2])
        writer.writerow(['control', 'pid', 'callback',\
            dt3sum /  no_iters * 1e-3, firstdt3])
        writer.writerow(['control', 'pid', 'callback+node',\
            dtsum /  no_iters * 1e-3, firstdt])
        f.close()

        print("Node and Pid calculated in ", dtsum /  no_iters * 1e-3, " ms")
        print("Pid path processing calculated in ", dt2sum / no_iters * 1e-3, " ms")
        print("Pid only calculated in ", dt3sum / no_iters * 1e-3, " ms")
        

    def test_mpc_callback(self):
        dtsum = 0
        dt2sum = 0
        dt3sum = 0
        firstdt = 0
        firstdt2 = 0
        firstdt3 = 0
        no_iters = 100

        position = Point2d()
        position.x = 0.0
        position.y = -1.0
        yaw = 0

        raw_path = [(1.5,0.0), (1.75,1.0), (2.0,2.0), (2.5,2.4), (3.0,2.8),\
            (3.5,3.15), (4.0,3.5), (4.25,4.25), (4.5,5.0), (4.7,5.5), (5.0, 6.0)]

        for i in range(no_iters):

            t0 = datetime.datetime.now() 
            rclpy.init()
            t1 = datetime.datetime.now()
            control_node = ControlNode()

            recv_path = PointArray()
            for point_tuple in raw_path:
                point = Point2d()
                point.x = point_tuple[0]
                point.y = point_tuple[1]
                recv_path.points.append(point)

            control_node.path_callback(recv_path)
            t2 = datetime.datetime.now()  
            control_node.mpc_callback(position, yaw)

            t3 = datetime.datetime.now()       
            dt = t3 - t0
            dt2 = t2 - t1
            dt3 = t3 - t2

            dtsum += dt.microseconds
            dt2sum += dt2.microseconds
            dt3sum += dt3.microseconds
            
            rclpy.shutdown()

            #print("acc: ", control_node.acceleration_command)
            #print("angle: ", control_node.steering_angle_command)
            #self.assertGreater(control_node.acceleration_command, 0)
            #self.assertLess(control_node.steering_angle_command, 0)
            if i == 0:
                firstdt = dtsum * 1e-3
                firstdt2 = dt2sum * 1e-3
                firstdt3 = dt2sum * 1e-3
                print("First Node and Mpc calculated in ", firstdt * 1e-3, " ms")
                print("First Mpc path processing calculated in ",\
                    firstdt2 * 1e-3, " ms")
                print("First Mpc only calculated in ", firstdt3 * 1e-3, " ms")
                

        f = open('control/test/control_measures.csv', 'a')
        writer = csv.writer(f)
        writer.writerow(['control', 'mpc', 'path_processing-'\
            + str(len(raw_path)) + 'pts', dt2sum /  no_iters * 1e-3, firstdt2])
        writer.writerow(['control', 'mpc', 'callback-' +\
            str(P.prediction_horizon) + 'ph', dt3sum /  no_iters * 1e-3, firstdt3])
        writer.writerow(['control', 'mpc', 'callback+node-' +\
            str(P.prediction_horizon) + 'ph', dtsum /  no_iters * 1e-3, firstdt])
        f.close()

        print("Node and Mpc calculated in ", dtsum /  no_iters * 1e-3, " ms")
        print("Mpc path processing calculated in ", dt2sum / no_iters * 1e-3, " ms")
        print("Mpc only calculated in ", dt3sum / no_iters * 1e-3, " ms")

    def test_mpc_optimization(self):
        rclpy.init()
        control_node = ControlNode()

        position = Point2d()
        position.x = 0.0
        position.y = -1.0
        yaw = 0

        raw_path = [(1.5,0.0), (1.75,1.0), (2.0,2.0), (2.5,2.4), (3.0,2.8), (3.5,3.15),\
            (4.0,3.5), (4.25,4.25), (4.5,5.0), (4.7,5.5), (5.0, 6.0)]
        recv_path = PointArray()
        for point_tuple in raw_path:
            point = Point2d()
            point.x = point_tuple[0]
            point.y = point_tuple[1]
            recv_path.points.append(point)

        control_node.path_callback(recv_path)
        control_node.mpc_callback(position, yaw, True)
        
        rclpy.shutdown()


if __name__ == '__main__':
    print("Running Tests...")
    unittest.main()
    

    

