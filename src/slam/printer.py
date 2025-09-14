import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ConeArray


class MapPrinterNode(Node):
    def __init__(self):
        super().__init__("map_printer_node")
        self.subscription = self.create_subscription(
            ConeArray, "/state_estimation/map", self.map_callback, 10
        )
        self.get_logger().info("MapPrinterNode has been started.")

    def map_callback(self, msg):
        cones = msg.cone_array

        formatted_cones = ", ".join(
            f"({cones[i].position.x}, {cones[i].position.y})"
            for i in range(0, len(cones))
        )
        self.get_logger().info(f"Cones: {formatted_cones}")


def main(args=None):
    rclpy.init(args=args)
    node = MapPrinterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
