from ackermann_msgs.msg import AckermannDriveStamped
from abc import ABC, abstractmethod

def create_abstraction_layer(node):
    """!
    @brief Creates the abstraction layer.
    @param node The node object.
    @return Abstraction layer object.
    """
    mode = node.get_parameter('mode').get_parameter_value().string_value

    if mode == "sim":
        return SimInterface(node)
    elif mode == "real":
        return CANInterface(node)
    else:
        raise ValueError("Invalid mode: {}".format(mode))

class AbstractionLayer(ABC):
    def __init__(self, node):
        self.node = node
        self.setup_vars()

        timer_period = 0.2  # seconds
        node.timer = node.create_timer(timer_period, self.timer_callback)
    
    @abstractmethod
    def setup_vars(self):
        pass

    @abstractmethod
    def timer_callback(self):
        pass

class SimInterface(AbstractionLayer):
    def __init__(self, node):
        super().__init__(node)

    def setup_vars(self):
        """!
        @brief Sets up the timer function calls.
        @param self The object pointer.
        """
        self.node.publisher_ = self.node.create_publisher(
            AckermannDriveStamped, "/cmd", 10
        )

    def timer_callback(self):
        """!
        @brief Sim publisher callback.
        @param self The object pointer.
        """
        node = self.node
        
        # Create a String message
        ack_msg = AckermannDriveStamped()

        # TODO: Set the minimum and maximum steering angle
        ack_msg.drive.steering_angle = node.steering_angle if not node.done else 0.
        # ack_msg.drive.steering_angle = node.steering_angle

        # Set car acceleration
        ack_msg.drive.speed = node.velocity if not node.done else -1.
        
        # Publish the message to the topic
        node.publisher_.publish(ack_msg)
        
        # Display the message on the console
        node.get_logger().info('Published to Simulator!')

class CANInterface(AbstractionLayer):
    def __init__(self, node):
        super().__init__(node)

    def setup_vars(self):
        pass

    def timer_callback(self):
        """!
        @brief CAN API callback.
        @param self The object pointer.
        """
        node = self.node
        node.get_logger().info('CAN API callback!')

