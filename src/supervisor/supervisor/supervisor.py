import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from custom_interfaces.msg import DataLogInfo1, WheelRPM

from message_filters import Subscriber, ApproximateTimeSynchronizer

import time
import subprocess
import signal
import os
import time

FILE_PATH = "/home/fsfeup/.vehicle_odometer"
WHEEL_DIAMETER = 0.406

class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')

        """
        ******************** Related to keeping nodes alive ********************
        """
        self.node_names_to_watch = ['ros_can', 'xsens_mti_node']
        self.node_start_cmds = {
            'ros_can': 'source install/setup.bash && ros2 run ros_can ros_can',
            'xsens_mti_node': 'source install/setup.bash && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py'
        }
        self.nodes_being_initialized = set()
        
        """
        ******************** General purpose checkup timer ********************
        """
        
        self.timer_period_milliseconds = 10
        self.create_timer(self.timer_period_milliseconds / 1000.0, self.check_up)

        """
        ******************** Related to recording rosbag ********************
        """
        self.rosbag_process = None
        self.master_topic = '/vehicle/data_log_info_1'
        self.record_rosbag_command = 'source install/setup.bash && ros2 bag record -s mcap --all'
        self.consecutive_ts_on_count = 0
        self.consecutive_ts_off_count = 0
        self.last_received_master_msg_time = 0

        # Subscribe to the master
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            DataLogInfo1,
            self.master_topic,
            self.master_callback,
            qos_profile
        )

        """
        ******************** Related to recording distance travelled ********************
        """
        # Subscribe wheel RPMs with a filter to synchronize them
        self.distance = 0.0
        self.last_received_time = 0.0
        self.fr_rpm_subscription = Subscriber(self, WheelRPM, '/vehicle/fr_rpm')
        self.fl_rpm_subscription = Subscriber(self, WheelRPM, '/vehicle/fl_rpm')
        self.ts = ApproximateTimeSynchronizer(
            [self.fr_rpm_subscription, self.fl_rpm_subscription],  # List of subscribers
            queue_size=10,
            slop=0.1  # seconds
        )
        self.ts.registerCallback(self.rpm_callback)


    """
    ******************** General purpose checkup ********************
    """
    def check_up(self):
        # Check if nodes are running and restart if necessary
        active_nodes_output = subprocess.check_output(['ros2', 'node', 'list'], text=True)
        active_nodes = set(active_nodes_output.splitlines())

        for node in active_nodes:
            if node[1:] in self.nodes_being_initialized:
                self.nodes_being_initialized.remove(node[1:])

        for name in self.node_names_to_watch:
            if f'/{name}' not in active_nodes and name not in self.nodes_being_initialized:
                self.get_logger().warn(f'Node "{name}" not found! Restarting...')
                self.restart_node(name)

        # Check if the rosbag process is running for too long
        if self.rosbag_process is not None:
            current_time = time.time()
            if current_time - self.last_received_master_msg_time > 2:
                self.get_logger().warn('No master message received for over 2 seconds.')
                self.stop_rosbag()
        

    """
    ******************** Related to keeping nodes alive ********************
    """

    def restart_node(self, node_name):
        cmd = self.node_start_cmds[node_name]
        subprocess.Popen(
            ['bash', '-c', cmd],
        )
        self.nodes_being_initialized.add(node_name)
        
    """
    ******************** Related to recording rosbag ********************
    """

    def master_callback(self, msg):
        self.last_received_master_msg_time = time.time()
        if self.rosbag_process is None:
            if (msg.ts_on):
                self.consecutive_ts_on_count += 1
            else:
                self.consecutive_ts_on_count = 0
            self.consecutive_ts_off_count = 0
        else:
            if (msg.ts_on):
                self.consecutive_ts_off_count = 0
            else:
                self.consecutive_ts_off_count += 1
            self.consecutive_ts_on_count = 0
        if self.should_start_recording(msg):
            self.start_recording_rosbag()
            self.read_distance()
        elif self.should_stop_recording(msg):
            self.stop_rosbag()
            self.write_distance(self.distance)

    def read_distance(self):
        try:
            with open(FILE_PATH, 'r') as f:
                self.distance = float(f.read())
        except FileNotFoundError:
            return

    def write_distance(self, total_distance):
        with open(FILE_PATH, 'w') as f:
            f.write(str(total_distance))
            f.flush()
            os.fsync(f.fileno())

    def rpm_callback(self, fr_msg, fl_msg):
        if self.last_received_time == 0.0:
            self.last_received_time = time.time()
            return
        current_time = time.time()
        time_diff = current_time - self.last_received_time
        self.distance += (fr_msg.rpm + fl_msg.rpm) / 2 * WHEEL_DIAMETER * 3.14159 / 60.0 * time_diff

    def should_start_recording(self, msg):
        if self.rosbag_process is not None:
            return False
        return self.consecutive_ts_on_count >= 2

    def should_stop_recording(self, msg):
        if self.rosbag_process is None:
            return False
        return self.consecutive_ts_off_count >= 6

    def start_recording_rosbag(self):
        self.get_logger().info('Starting rosbag recording...')
        cmd = self.record_rosbag_command
        self.rosbag_process = subprocess.Popen(['bash', '-c', cmd])
    
    def stop_rosbag(self):
        self.get_logger().info('Stopping rosbag recording...')
        self.rosbag_process.send_signal(signal.SIGINT)
        self.rosbag_process = None


def main(args=None):
    rclpy.init(args=args)
    node = Supervisor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
