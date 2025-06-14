import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from custom_interfaces.msg import WheelRPM
from std_msgs.msg import Float32MultiArray
import numpy as np

WHEEL_RADIUS = 0.203  
WHEEL_BASE = 1.6      

class InfrastructureDesign(Node):
    def __init__(self):
        super().__init__('infrastructure_design_node')
        
        self.metrics_pub = self.create_publisher(Float32MultiArray, '/infrastructure/metrics', 10)

        self.rl_rpm_subscription = Subscriber(self, WheelRPM, "/vehicle/rl_rpm")
        self.rr_rpm_subscription = Subscriber(self, WheelRPM, "/vehicle/rr_rpm")
        
        self.wss_sensors = ApproximateTimeSynchronizer(
            [self.rl_rpm_subscription, self.rr_rpm_subscription],
            10,
            0.1,
            allow_headerless=True
        )
        self.wss_sensors.registerCallback(self.wss_callback)

    def wss_callback(self, rl_rpm_msg, rr_rpm_msg):
        fl_rpm = rl_rpm_msg.fl_rpm
        rl_rpm = rl_rpm_msg.rl_rpm
        fr_rpm = rr_rpm_msg.fr_rpm
        rr_rpm = rr_rpm_msg.rr_rpm

        metrics = self.calculate_wheel_rpm_metrics(fl_rpm, rl_rpm, fr_rpm, rr_rpm)
        
        msg = Float32MultiArray()
        msg.data = [
            metrics["average_rpm"],
            metrics["min_rpm"],
            metrics["max_rpm"],
            metrics["left_avg_rpm"],
            metrics["right_avg_rpm"],
            metrics["front_avg_rpm"],
            metrics["rear_avg_rpm"],
            metrics["front_rear_diff"],
            metrics["left_right_diff"],
            metrics["rpm_variance"],
            metrics["rpm_stddev"],
            metrics["average_velocity"],
            *metrics["wheel_velocities"]  
        ]
        self.metrics_pub.publish(msg)

    @staticmethod
    def calculate_wheel_rpm_metrics(fl_rpm, rl_rpm, fr_rpm, rr_rpm):
        rpms = [fl_rpm, rl_rpm, fr_rpm, rr_rpm]
        left_rpms = [fl_rpm, rl_rpm]
        right_rpms = [fr_rpm, rr_rpm]
        front_rpms = [fl_rpm, fr_rpm]
        rear_rpms = [rl_rpm, rr_rpm]

        omegas = [rpm * 2 * np.pi / 60 for rpm in rpms]
        velocities = [omega * WHEEL_RADIUS for omega in omegas]

        avg_velocity = sum(velocities) / 4

        metrics = {
            "average_rpm": sum(rpms) / 4,
            "min_rpm": min(rpms),
            "max_rpm": max(rpms),
            "left_avg_rpm": sum(left_rpms) / 2,
            "right_avg_rpm": sum(right_rpms) / 2,
            "front_avg_rpm": sum(front_rpms) / 2,
            "rear_avg_rpm": sum(rear_rpms) / 2,
            "front_rear_diff": (sum(front_rpms) / 2) - (sum(rear_rpms) / 2),
            "left_right_diff": (sum(left_rpms) / 2) - (sum(right_rpms) / 2),
            "rpm_variance": float(np.var(rpms)),
            "rpm_stddev": float(np.std(rpms)),
            "wheel_velocities": velocities,
            "average_velocity": avg_velocity,
        }
        return metrics

def main(args=None):
    rclpy.init(args=args)
    node = InfrastructureDesign()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()