#!/usr/bin/env python3
import subprocess
import time
import os
import signal
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from custom_interfaces.msg import LapSummary, LapCurrent

class SimEvaluator(Node):
    def __init__(self, target_laps=10, timeout=300):
        super().__init__('sim_evaluator')
        self.target_laps = target_laps
        self.timeout = timeout
        self.start_time = time.time()
        self.lap_results = []
        self.done = False

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.summary_sub = self.create_subscription(
            LapSummary,
            '/invictasim/statistics/lap_summary',
            self.summary_callback,
            qos
        )
        self.current_sub = self.create_subscription(
            LapCurrent,
            '/invictasim/statistics/lap_current',
            self.current_callback,
            10
        )
        self.last_current_lap = 0

    def current_callback(self, msg):
        if msg.lap_number != self.last_current_lap:
            self.last_current_lap = msg.lap_number
            print(f"--> In lap {msg.lap_number} (elapsed: {time.time() - self.start_time:.1f}s)", flush=True)

    def summary_callback(self, msg):
        self.lap_results = list(msg.rows)
        latest = msg.rows[-1]
        print(f"[Lap {latest.lap_number}] time={latest.time:.4f}s, cones={latest.cones_hit}, "
              f"avg_v={latest.avg_velocity:.2f} km/h, max_v={latest.max_velocity:.2f} km/h, "
              f"avg_err={latest.avg_tracking_error_distance:.4f}m, max_err={latest.max_tracking_error_distance:.4f}m", flush=True)
        if len(self.lap_results) >= self.target_laps:
            self.done = True

def kill_existing():
    patterns = ["invictasim", "planning", "node_control"]
    for pat in patterns:
        subprocess.run(["pkill", "-2", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    for pat in patterns:
        subprocess.run(["pkill", "-9", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)

def run_simulation(target_laps=10, timeout=300):
    print("Cleaning up any existing processes...")
    kill_existing()

    print("Launching invictasim, planning, and control...")
    env = os.environ.copy()
    
    p_sim = subprocess.Popen(["ros2", "launch", "invictasim", "invictasim.launch.py"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    time.sleep(2)
    p_plan = subprocess.Popen(["ros2", "launch", "planning", "planning.launch.py"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    time.sleep(1)
    p_ctrl = subprocess.Popen(["ros2", "launch", "control", "control.launch.py"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)

    rclpy.init()
    node = SimEvaluator(target_laps=target_laps, timeout=timeout)
    start_t = time.time()

    try:
        while rclpy.ok() and not node.done and (time.time() - start_t) < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if p_sim.poll() is not None:
                print("Warning: invictasim exited unexpectedly!")
                break
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Stopping processes...")
        kill_existing()

    print("\n--- RESULTS TABLE ---")
    header = ["cones_hit", "total_time", "best_time", "avg_velocity", "max_velocity", "avg_tracking_error_distance", "max_tracking_error_distance", "avg_velocity_error", "max_velocity_error"]
    print("\t".join(header))
    for r in node.lap_results:
        print(f"{r.lap_number}\n{r.time}\n{r.cones_hit}\n{r.total_time}\n{r.best_time}\n{r.avg_velocity}\n{r.max_velocity}\n{r.avg_tracking_error_distance}\n{r.max_tracking_error_distance}\n{r.avg_velocity_error}\n{r.max_velocity_error}")

    return node.lap_results

if __name__ == "__main__":
    target = 10
    if len(sys.argv) > 1:
        target = int(sys.argv[1])
    run_simulation(target_laps=target)
