from control.control_node import ControlNode
from custom_interfaces.msg import PointArray, Point2d
import rclpy
import unittest
import numpy as np
import time

from control.config import Params
from control.mpc import MPC
from control.mpc_utils import (
    get_ref_trajectory,
    get_linear_model_matrices,
    optimize,
)

from utils import save_exec_time

P = Params()

class TestUtilsMethods(unittest.TestCase):

    def test_pid_callback(self):
        dt = []
        dt2 = []
        dt3 = []
        no_iters = 100

        position = Point2d()
        position.x = 0.0
        position.y = -1.0
        yaw = 0

        raw_path = [(1.5,0.0), (1.75,1.0), (2.0,2.0), (2.5,2.4), (3.0,2.8),\
            (3.5,3.15), (4.0,3.5), (4.25,4.25), (4.5,5.0), (4.7,5.5), (5.0, 6.0)]

        for i in range(no_iters):
            t0 = time.perf_counter() 
            rclpy.init()
            t1 = time.perf_counter()   
            control_node = ControlNode()

            recv_path = PointArray()
            for point_tuple in raw_path:
                point = Point2d()
                point.x = point_tuple[0]
                point.y = point_tuple[1]
                recv_path.points.append(point)

            control_node.path_callback(recv_path)
            t2 = time.perf_counter() 
            control_node.pid_callback(position, yaw)

            t3 = time.perf_counter()      
            dt.append(float(t3 - t0))
            dt2.append(float(t2 - t1))
            dt3.append(float(t3 - t2)) 

            rclpy.shutdown()

        save_exec_time('control', 'pid', 'path_processing-' +\
            str(len(raw_path)) + 'pts', np.array(dt2).mean() * 1000)
        save_exec_time('control', 'pid', 'callback',\
            np.array(dt3).mean() * 1000)
        save_exec_time('control', 'pid', 'callback+node',\
            np.array(dt).mean() * 1000)

        print("Node and Pid calculated in ", np.array(dt).mean() * 1000, " ms. First time was {}".format(dt[0] * 1000))
        print("Pid path processing calculated in ", np.array(dt2).mean() * 1000, " ms. First time was {}".format(dt2[0] * 1000))
        print("Pid only calculated in ", np.array(dt3).mean() * 1000, " ms. First time was {}".format(dt3[0] * 1000))
        

    def test_mpc_callback(self):
        dt = []
        dt2 = []
        dt3 = []
        no_iters = 100

        position = Point2d()
        position.x = 0.0
        position.y = -1.0
        yaw = 0

        raw_path = [(1.5,0.0), (1.75,1.0), (2.0,2.0), (2.5,2.4), (3.0,2.8),\
            (3.5,3.15), (4.0,3.5), (4.25,4.25), (4.5,5.0), (4.7,5.5), (5.0, 6.0)]

        for i in range(no_iters):

            t0 = time.perf_counter() 
            rclpy.init()
            t1 = time.perf_counter()
            control_node = ControlNode()

            recv_path = PointArray()
            for point_tuple in raw_path:
                point = Point2d()
                point.x = point_tuple[0]
                point.y = point_tuple[1]
                recv_path.points.append(point)

            control_node.path_callback(recv_path)
            t2 = time.perf_counter()  
            control_node.mpc_callback(position, yaw)
            t3 = time.perf_counter()       
            dt.append(t3 - t0)
            dt2.append(t2 - t1)
            dt3.append(t3 - t2) 
            
            rclpy.shutdown()
                
        save_exec_time('control', 'mpc', 'path_processing-' +\
            str(len(raw_path)) + 'pts', np.array(dt2).mean() * 1000)
        save_exec_time('control', 'mpc', 'callback-' +\
            str(P.prediction_horizon) + 'ph', np.array(dt3).mean() * 1000)
        save_exec_time('control', 'mpc', 'callback+node-' +\
            str(P.prediction_horizon) + 'ph', np.array(dt).mean() * 1000)

        print("Node and Mpc calculated in ", np.array(dt).mean() * 1000, " ms. First time was {}".format(dt[0] * 1000))
        print("Mpc path processing calculated in ", np.array(dt2).mean() * 1000, " ms. First time was {}".format(dt2[0] * 1000))
        print("Mpc only calculated in ", np.array(dt3).mean() * 1000, " ms. First time was {}".format(dt3[0] * 1000))


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

        # Set mpc
        current_action = np.array(
            [control_node.acceleration_command, control_node.steering_angle_actual])
        current_state = np.array([position.x, position.y,\
            control_node.velocity_actual, yaw])

        mpc = MPC(current_action, current_state, control_node.path,\
            control_node.old_closest_index)
        
        curr_state = np.array([0, 0, mpc.state[2], 0])

        A, B, C = get_linear_model_matrices(curr_state, mpc.action)
        
        x_target, u_target, mpc.closest_ind = get_ref_trajectory(
            mpc.state, mpc.path, P.VEL, old_ind=mpc.old_closest_ind
        )

        # Test
        dt = []
        no_iters = 100
        for _ in range(no_iters):

            t0 = time.perf_counter()  

            _, _ = optimize(
                A,
                B,
                C,
                curr_state,
                x_target,
                u_target,
                verbose=False
            )

            t1 = time.perf_counter()
            dt.append(t1 - t0)

        save_exec_time('control', 'mpc', 'optimization_step-' +\
            str(P.prediction_horizon) + 'ph', np.array(dt).mean() * 1000)
        
        print("Average optimization step is ", np.array(dt).mean() * 1000, " ms. First time was {}".format(dt[0] * 1000))
        
        rclpy.shutdown()


if __name__ == '__main__':
    print("Running Tests...")
    unittest.main()
    

    

