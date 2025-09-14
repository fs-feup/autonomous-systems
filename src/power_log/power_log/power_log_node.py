#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import psutil
from std_msgs.msg import Float32, Float32MultiArray, Int64MultiArray
from custom_interfaces.msg import ProcessUsage, ProcessUsageArray
import csv
import signal
import datetime
import sys
import time


class PowerNode(Node):
    def __init__(self):
        super().__init__("power_log_node")
        self.cpu_publisher = self.create_publisher(
            Float32MultiArray, "cpu_core_usage", 10
        )
        self.memory_publisher = self.create_publisher(Float32, "memory_usage", 10)
        self.temperature_publisher = self.create_publisher(
            Float32, "cpu_temperature", 10
        )
        self.processes_usage_publisher = self.create_publisher(
            ProcessUsageArray, "processes_usage", 10
        )
        self.create_timer(1.0, self.timer_callback)

        self.metrics_list = []
        self.processes_usage = []

        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        """!
        Writes metrics to csv and exits when Ctrl+C is pressed.

        Args:
            sig (int): Signal number.
            frame (frame): Current stack frame.
        """

        if self.metrics_list:  # Check if the list is not empty
            finish_time = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
            datetime_filename = f"power_log_{finish_time}.csv"
            self.metrics_to_csv(
                self.metrics_list, "performance/power_log_metrics/" + datetime_filename
            )
        sys.exit(0)

    def metrics_to_csv(self, metrics, filename):
        """!
        Converts metrics to csv and writes them to a file.

        Args:
            metrics (list): List of metrics dictionaries.
            filename (str): Name of the file to write the metrics to.
        """

        # Add 'time' key to each metric
        start_time = metrics[0]["timestamp"]
        for metric in metrics:
            elapsed_time = (metric["timestamp"] - start_time).total_seconds()
            metric["time"] = elapsed_time  # Add/Update 'time' key with elapsed time

        # Write metrics to csv
        with open(filename, "w", newline="") as csvfile:
            fieldnames = metrics[0].keys()
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(metrics)

    def get_process_usage(self, top_n=20):
        # First call to cpu_percent initializes counters
        for proc in psutil.process_iter(["pid", "name"]):
            try:
                proc.cpu_percent(interval=None)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

        processes = []

        for proc in psutil.process_iter(["pid", "name"]):
            try:
                cpu = proc.cpu_percent(interval=None)
                mem = proc.memory_info().rss / (1024 * 1024)  # MB
                processes.append((cpu, mem, proc.info["pid"], proc.info["name"]))
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        # Sort by CPU usage
        processes.sort(key=lambda x: x[0], reverse=True)

        return processes[:top_n]  # Return top N processes

    def pc_stats(self):
        """!
        Returns a dictionary with the CPU core usage, memory usage and CPU temperature.
        """
        temps = psutil.sensors_temperatures()
        cpu_temp = None
        if "k10temp" in temps:
            for entry in temps["k10temp"]:
                if entry.label == "Tctl":
                    cpu_temp = entry.current
                    break

        return {
            "cpu_core_usage": psutil.cpu_percent(interval=0.1, percpu=True),
            "memory_usage": psutil.virtual_memory().percent,
            "cpu_temperature": cpu_temp,
        }

    def timer_callback(self):
        """!
        Callback function that publishes CPU core usage, memory usage and CPU temperature.
        """
        stats = self.pc_stats()
        processes = self.get_process_usage()

        cpu_core_usage_msg = Float32MultiArray()
        cpu_core_usage_msg.data = stats["cpu_core_usage"]
        self.cpu_publisher.publish(cpu_core_usage_msg)

        processes_usage_msg = ProcessUsageArray()
        processes_usage_msg.header.stamp = self.get_clock().now().to_msg()
        processes_usage_msg.process_usages = []
        for cpu, mem, pid, name in processes:
            process_usage = ProcessUsage()
            process_usage.header.stamp = processes_usage_msg.header.stamp
            process_usage.usage = cpu
            process_usage.memory = mem
            process_usage.process_name = name
            processes_usage_msg.process_usages.append(process_usage)

        self.processes_usage_publisher.publish(processes_usage_msg)

        memory_msg = Float32()
        memory_msg.data = stats["memory_usage"]
        self.memory_publisher.publish(memory_msg)

        temperature_msg = Float32()
        temperature_msg.data = stats["cpu_temperature"]
        self.temperature_publisher.publish(temperature_msg)

        metrics = {
            "timestamp": datetime.datetime.now(),
            "memory_usage": stats["memory_usage"],
            "cpu_temperature": stats["cpu_temperature"],
        }

        for i in range(16):
            metrics[f"cpu_core_{i+1}_usage"] = (
                stats["cpu_core_usage"][i] if i < len(stats["cpu_core_usage"]) else 0
            )

        self.metrics_list.append(metrics)


def main(args=None):
    rclpy.init(args=args)
    node = PowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
