import rclpy
from rclpy.node import Node

from custom_interfaces.msg import PointArray

STEER_CONTROL = 1
SPEED_CONTROL = 1
KEEP_GOING = 0

class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
        self.control_info = (STEER_CONTROL, SPEED_CONTROL, KEEP_GOING)

        self.subscription = self.create_subscription(
            PointArray,
            'path_mock',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % ' '.join([str(point) for point in msg.points]))


def main(args=None):
    rclpy.init(args=args)

    control_node = ControlNode()

    rclpy.spin(control_node)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()