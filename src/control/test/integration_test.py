import numpy as np
import rclpy
import unittest
import time
from custom_interfaces.msg import PointArray, Point2d, VcuCommand, Pose
from control.control_node import ControlNode
from eufs_msgs.msg import CanState
from rclpy.executors import MultiThreadedExecutor

from utils import save_exec_time

class IntegrationTest(unittest.TestCase):

    count = 0
    start_time : float = 0.0
    times = []

    #!
    # @brief Test node callback
    # Test if the control node is able to receive messages and the control node's
    # execution times
    # @param msg Message received
    def message_callback(self, msg):
        end_time = time.perf_counter()
        IntegrationTest.received_control = msg
        IntegrationTest.count += 1
        if IntegrationTest.count != 1:
            difference : float = end_time - IntegrationTest.start_time
            IntegrationTest.times.append(difference)
        if IntegrationTest.count >= 100:
            avg_exec_time = np.array(IntegrationTest.times).mean()
            save_exec_time('Control', 'All', '20points', avg_exec_time * 1000)
            rclpy.shutdown()
        else:
            self.vl_publisher.publish(self.vl_msg)
            self.planning_publisher.publish(self.point_array_msg)
            IntegrationTest.start_time = time.perf_counter()


    #! 
    # @brief Setup for test
    def setUp(self):
        rclpy.init()

        # Messages
        self.point_array_msg  : PointArray = PointArray()
        self.vl_msg : Pose = Pose()

        # Nodes
        self.receiver_node = rclpy.create_node("can_receiver")
        self.control_node = ControlNode()

        # Publishers and subscribers
        self.planning_publisher = self.receiver_node.create_publisher(\
            PointArray, "/planning_local", 10)
        self.vl_publisher = self.receiver_node.create_publisher(\
            Pose, "/vehicle_localization", 10)
        self.can_subscriber = self.receiver_node.create_subscription(\
            VcuCommand,      
            "/cmd",
            self.message_callback,
            10
        )


    #!
    # @brief Setup path and pose and launch nodes
    def test_1(self):

        # Path
        point1 = Point2d(x = 1.5, y = 0.0)
        self.point_array_msg.points.append(point1)
        point2 = Point2d(x = 1.75, y = 1.0)
        self.point_array_msg.points.append(point2)
        point3 = Point2d(x = 2.0, y = 2.0)
        self.point_array_msg.points.append(point3)
        point4 = Point2d(x = 2.5, y = 2.4)
        self.point_array_msg.points.append(point4)
        point5 = Point2d(x = 3.0, y = 2.2)
        self.point_array_msg.points.append(point5)
        point6 = Point2d(x = 3.5, y = 2.2)
        self.point_array_msg.points.append(point6)
        point7 = Point2d(x = 4.0, y = 2.0)
        self.point_array_msg.points.append(point7)
        point8 = Point2d(x = 4.3, y = 2.5)
        self.point_array_msg.points.append(point8)
        point9 = Point2d(x = 4.5, y = 3.0)
        self.point_array_msg.points.append(point9)
        point10 = Point2d(x = 4.7, y = 3.3)
        self.point_array_msg.points.append(point10)
        point11 = Point2d(x = 5.0, y = 3.6)
        self.point_array_msg.points.append(point11)
        point12 = Point2d(x = 5.3, y = 4.0)
        self.point_array_msg.points.append(point12)
        point13 = Point2d(x = 5.6, y = 4.3)
        self.point_array_msg.points.append(point13)
        point14 = Point2d(x = 5.9, y = 4.6)
        self.point_array_msg.points.append(point14)
        point15 = Point2d(x = 6.4, y = 5.0)
        self.point_array_msg.points.append(point15)
        point16 = Point2d(x = 7.0, y = 5.3)
        self.point_array_msg.points.append(point16)
        point17 = Point2d(x = 7.5, y = 5.2)
        self.point_array_msg.points.append(point17)
        point18 = Point2d(x = 7.9, y = 4.9)
        self.point_array_msg.points.append(point18)
        point19 = Point2d(x = 8.3, y = 4.6)
        self.point_array_msg.points.append(point19)
        point20 = Point2d(x = 8.6, y = 4.3)
        self.point_array_msg.points.append(point20)

        # Localisation
        pos = Point2d()
        pos.x = 0.0
        pos.y = -1.0
        self.vl_msg = Pose(position=pos, theta = 0.0,\
            steering_angle = 0.0, velocity = 0.0)
        
        self.control_node.state = CanState.AS_DRIVING
        self.control_node.mission = CanState.AMI_AUTOCROSS

        # Launch nodes
        executor = MultiThreadedExecutor()
        executor.add_node(self.control_node)     
        executor.add_node(self.receiver_node)
        executor.spin()


if __name__ == '__main__':
    print("Integration Tests...")
    unittest.main()