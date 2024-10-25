import rclpy
from rclpy.node import Node
from pacsim.msg import Wheels
from std_msgs.msg import Float32


class CarSpeedPublisher(Node):

    def __init__(self):
        super().__init__("car_speed_publisher")

        self.subscription = self.create_subscription(
            Wheels, "/pacsim/wheelspeeds", self.wheelspeed_callback, 10
        )

        self.speed_publisher = self.create_publisher(Float32, "car_speed", 10)
        self.rpm_publisher = self.create_publisher(Float32, "rpm", 10)

        self.timer = self.create_timer(0.1, self.publish_car_speed)

        self.wheel_radius = 0.26
        self.avg_wheelspeed = 0.0
    
    def wheelspeed_callback(self, msg):
        fl_speed = msg.fl
        fr_speed = msg.fr
        rl_speed = msg.rl
        rr_speed = msg.rr

        self.avg_wheelspeed = (fl_speed + fr_speed + rl_speed + rr_speed) / 4.0

    def publish_car_speed(self):
        car_speed = 2 * 3.14 * self.wheel_radius * self.avg_wheelspeed / 60

        speed_msg = Float32()
        speed_msg.data = car_speed
        rpm_msg = Float32()
        rpm_msg.data = self.avg_wheelspeed
        self.speed_publisher.publish(speed_msg)
        self.rpm_publisher.publish(rpm_msg)

        self.get_logger().info(
            f"Car Speed: {speed_msg.data:.2f} m/s  RPM: {rpm_msg.data:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CarSpeedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
