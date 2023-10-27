import rclpy
import unittest
import time
from custom_interfaces.msg import PointArray, Point2d, VcuCommand, Pose
from control.control_node import ControlNode
from eufs_msgs.msg import CanState
from rclpy.executors import MultiThreadedExecutor


class IntegrationTest(unittest.TestCase):
    # Params in config set to Ads_dv message type

    planning_sender = None
    can_receiver = None
    control_test = None
    point_array_msg = None
    received_control = None
    planning_publisher = None
    count = 0
         
    def message_callback(self, msg):
        if self.received_control is None:
            self.received_control = msg
            return
        self.received_control = msg
        rclpy.shutdown()

    def setUp(self):
        rclpy.init()

        self.planning_sender = rclpy.create_node('planning_sender')
        self.vl_sender = rclpy.create_node('vl_sender')
        self.can_receiver = rclpy.create_node("can_receiver")

        self.planning_publisher = self.planning_sender.create_publisher(\
            PointArray, "/planning_local", 10)
        self.vl_publisher = self.vl_sender.create_publisher(\
            Pose, "/vehicle_localization", 10)

        self.can_subscriber = self.can_receiver.create_subscription(\
            VcuCommand,      
            "/cmd",
            self.message_callback,
            10
        )

        self.control_test = ControlNode()
        
    
    def tearDown(self):
        self.planning_sender.destroy_node()
        self.vl_sender.destroy_node()
        self.can_receiver.destroy_node()
        self.control_test.destroy_node()

        #rclpy.shutdown()

    def test_1(self):
        point_array_msg = PointArray()
        point1 = Point2d(x = 1.5, y = 0.0)
        point_array_msg.points.append(point1)

        point2 = Point2d(x = 1.75, y = 1.0)
        point_array_msg.points.append(point2)

        point3 = Point2d(x = 2.0, y = 2.0)
        point_array_msg.points.append(point3)

        point4 = Point2d(x = 2.5, y = 2.4)
        point_array_msg.points.append(point4)

        point5 = Point2d(x = 3.0, y = 2.2)
        point_array_msg.points.append(point5)

        point6 = Point2d(x = 3.5, y = 2.2)
        point_array_msg.points.append(point6)

        point7 = Point2d(x = 4.0, y = 2.0)
        point_array_msg.points.append(point7)

        point8 = Point2d(x = 4.3, y = 2.5)
        point_array_msg.points.append(point8)

        point9 = Point2d(x = 4.5, y = 3.0)
        point_array_msg.points.append(point9)

        point10 = Point2d(x = 4.7, y = 3.3)
        point_array_msg.points.append(point10)

        point11 = Point2d(x = 5.0, y = 3.6)
        point_array_msg.points.append(point11)

        point12 = Point2d(x = 5.3, y = 4.0)
        point_array_msg.points.append(point12)

        point13 = Point2d(x = 5.6, y = 4.3)
        point_array_msg.points.append(point13)

        point14 = Point2d(x = 5.9, y = 4.6)
        point_array_msg.points.append(point14)

        point15 = Point2d(x = 6.4, y = 5.0)
        point_array_msg.points.append(point15)

        point16 = Point2d(x = 7.0, y = 5.3)
        point_array_msg.points.append(point16)

        point17 = Point2d(x = 7.5, y = 5.2)
        point_array_msg.points.append(point17)

        point18 = Point2d(x = 7.9, y = 4.9)
        point_array_msg.points.append(point18)

        point19 = Point2d(x = 8.3, y = 4.6)
        point_array_msg.points.append(point19)

        point20 = Point2d(x = 8.6, y = 4.3)
        point_array_msg.points.append(point20)

        time.sleep(2)

        self.control_test.get_logger().info(
            "[control] (Publishing Planning Test with Size {}".format(
                len(point_array_msg.points)
            )
        )
        pos = Point2d()
        pos.x = 0.0
        pos.y = -1.0
        vl_msg = Pose(position=pos, theta = 0.0,\
            steering_angle = 0.0, velocity = 0.0)

        self.control_test.state = CanState.AS_DRIVING
        self.control_test.mission = CanState.AMI_AUTOCROSS
        
        self.vl_publisher.publish(vl_msg)
        self.planning_publisher.publish(point_array_msg)
        

        executor = MultiThreadedExecutor()
        executor.add_node(self.vl_sender)
        executor.add_node(self.control_test)     
        executor.add_node(self.planning_sender)
        executor.add_node(self.can_receiver)
        
        executor.spin()

        self.assertLess(self.received_control.steering_angle_request, 0)
        self.assertGreater(self.received_control.axle_torque_request, 0)

if __name__ == '__main__':
    print("Integration Tests...")
    unittest.main()