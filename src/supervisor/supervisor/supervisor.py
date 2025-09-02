import re
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from custom_interfaces.msg import DataLogInfo1

import subprocess
import signal
import os
import time

class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        self.creation_time = time.time()

        """
        ******************** Related to keeping nodes alive ********************
        """
        self.node_names_to_watch = ['ros_can', 'xsens_mti_ros2_driver'] # or xsens_mti_ros2_driver
        self.node_start_cmds = {
            'ros_can': 'source install/setup.bash && ros2 run ros_can ros_can',
            'xsens_mti_ros2_driver': 'cd ~/Xsens_MTi_ROS_Driver_and_Ntrip_Client && source ./install/setup.bash && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py'
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
        self.bags_dir = os.path.join(os.getcwd(), "bags")
        os.makedirs(self.bags_dir, exist_ok=True)  # Creates if missing, does nothing if exists
        self.get_logger().info(f"Dir: {self.bags_dir}")
        self.record_rosbag_command = 'source install/setup.bash && ros2 bag record -s mcap -a --exclude /lidar_points'
        self.mission = "Unknown"
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
    ******************** General purpose checkup ********************
    """
    def check_up(self):
        # Check if nodes are running and restart if necessary
        active_nodes_output = subprocess.check_output(['ros2', 'node', 'list'], text=True)
        active_nodes = set(active_nodes_output.splitlines())

        for node in active_nodes:
            if node[1:] in self.nodes_being_initialized:
                self.nodes_being_initialized.remove(node[1:])
        current_time = time.time()
        for name in self.node_names_to_watch:
            if (f'/{name}' not in active_nodes ) and (name not in self.nodes_being_initialized) and (current_time - self.creation_time > 10):
                self.get_logger().warn(f'Node "{name}" not found! Restarting...')
                self.restart_node(name)

        # Check if the rosbag process is running for too long
        if self.rosbag_process is not None:
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
    def convert_mission(self,msg):
        if msg.mission is None:
            self.mission = "Unknown"
            return
        if msg.mission == 0:
            self.mission = "Manual"
        elif msg.mission == 1:
            self.mission = "Acceleration"
        elif msg.mission == 2:
            self.mission = "Skidpad"
        elif msg.mission == 3:
            self.mission = "Autocross"
        elif msg.mission == 4:
            self.mission = "Trackdrive"
        elif msg.mission == 5:
            self.mission = "EBS Test"
        elif msg.mission == 6:
            self.mission = "Inspection"
        elif msg.mission == 7:
            self.mission = "None"
        else:
            self.mission = "Unknown"

    def master_callback(self, msg):
        self.last_received_master_msg_time = time.time()
        if msg.mission is not None:
            self.convert_mission(msg)
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
        elif self.should_stop_recording(msg):
            self.stop_rosbag()

    def should_start_recording(self, msg):
        if self.rosbag_process is not None:
            return False
        return self.consecutive_ts_on_count >= 2

    def should_stop_recording(self, msg):
        if self.rosbag_process is None:
            return False
        return self.consecutive_ts_off_count >= 6

    def get_rosbag_naming(self):
        # Folder where rosbags are stored (adjust if needed)
        rosbag_dir = self.bags_dir
        # Regex to match names like "Skidpad 1", "Skidpad 2", etc.
        pattern = re.compile(rf"^{re.escape(self.mission)}\s+(\d+)\s*$")


        max_num = 0
        for name in os.listdir(rosbag_dir):
            if os.path.isdir(os.path.join(rosbag_dir, name)):
                match = pattern.match(name)
                if match:
                    num = int(match.group(1))
                    max_num = max(max_num, num)

        # Next available number
        next_num = max_num + 1
        return f"{self.mission} {next_num}"

    def start_recording_rosbag(self):
        rosbag_name = self.get_rosbag_naming()
        save_path = os.path.join(self.bags_dir, rosbag_name)
        print("Save path", save_path)

        self.get_logger().info(f'Starting rosbag recording: {save_path}')
        cmd = f'{self.record_rosbag_command} -o "{save_path}"'
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