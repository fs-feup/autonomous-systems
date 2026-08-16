# pylint: skip-file
# mypy: ignore-errors
"""Captures where and how a run leaves the track.

The weight campaign showed departures that correlate with no parameter and affect both
controllers, so the question is no longer "which weight" but "what was the car told to do, and
where". This records the vehicle pose, speed and reference at the moment the tracking error
starts to diverge, over repeated runs, so a location pattern (always the same corner => a
reference or geometry fault) can be told apart from a scatter (=> a timing or state fault).

Usage:  python3 capture_failure.py --runs 6 [--controller supermpc]
"""
import argparse
import csv
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ControlCommand, ControlStatistics, Pose


class Watcher(Node):
    def __init__(self, threshold):
        super().__init__("failure_watcher")
        self.threshold = threshold
        self.pose = (float("nan"), float("nan"), float("nan"))
        self.speed = float("nan")
        self.desired = float("nan")
        self.error = 0.0
        # The whole point is the state at ONSET, not after the car has already left, so the
        # first crossing is latched and never overwritten.
        self.onset = None
        self.trace = []
        self.create_subscription(ControlStatistics,
                                 "/invictasim/statistics/control_statistics", self._stats, 10)
        self.create_subscription(Pose, "/invictasim/state_estimation/vehicle_pose",
                                 self._pose, 10)
        # What the controller actually ASKED for. A car sitting at zero speed while being
        # commanded full throttle is stuck in the simulator; one being commanded zero throttle
        # is being told to stop, which is a fault in the control path.
        self.throttle = float("nan")
        self.steering = float("nan")
        self.create_subscription(ControlCommand, "/control/command", self._cmd, 10)

    def _cmd(self, msg):
        self.throttle = msg.throttle_rl
        self.steering = msg.steering

    def _pose(self, msg):
        self.pose = (msg.x, msg.y, msg.theta)

    def _stats(self, msg):
        self.error = abs(msg.tracking_error)
        self.speed = msg.current_velocity
        self.desired = msg.desired_velocity
        self.trace.append((time.time(), self.pose[0], self.pose[1], self.speed,
                           self.desired, self.error, self.throttle, self.steering))
        if self.onset is None and self.error > self.threshold:
            self.onset = {"x": self.pose[0], "y": self.pose[1], "yaw": self.pose[2],
                          "speed": self.speed, "desired": self.desired, "error": self.error}


def kill_all():
    for pattern in ("lib/invictasim/invictasim", "ros2 launch invictasim",
                    "node_control", "lib/planning/planning"):
        subprocess.run(["pkill", "-9", "-f", pattern],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def spawn(cmd, path, env=None):
    log = open(path, "w")
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, preexec_fn=os.setsid,
                            env=env or os.environ.copy()), log


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=6)
    parser.add_argument("--threshold", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=70.0)
    parser.add_argument("--out", default="/home/ws/src/control/test/failure_onsets.csv")
    parser.add_argument("--log-dir", default="/tmp/failure_logs")
    args = parser.parse_args()
    os.makedirs(args.log_dir, exist_ok=True)

    rclpy.init()
    watcher = Watcher(args.threshold)
    rows = []
    try:
        for run in range(args.runs):
            watcher.onset = None
            watcher.error = 0.0
            watcher.trace = []
            kill_all()
            env = os.environ.copy()
            env["SDL_VIDEODRIVER"] = "x11"
            sim, sl = spawn(["ros2", "launch", "invictasim", "invictasim.launch.py"],
                            os.path.join(args.log_dir, "sim.log"), env)
            time.sleep(14.0)
            plan, pl = spawn(["ros2", "run", "planning", "planning"],
                             os.path.join(args.log_dir, "plan_%d.log" % run))
            time.sleep(3.0)
            ctrl, cl = spawn(["ros2", "run", "control", "node_control"],
                             os.path.join(args.log_dir, "ctrl_%d.log" % run))

            deadline = time.time() + args.timeout
            while time.time() < deadline:
                rclpy.spin_once(watcher, timeout_sec=0.2)
                if watcher.onset is not None and watcher.error > 8.0:
                    break

            for proc in (ctrl, plan, sim):
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
            for handle in (cl, pl, sl):
                handle.close()
            kill_all()

            if watcher.onset:
                o = watcher.onset
                print("run %d: DIVERGED at (%.1f, %.1f) yaw=%.2f  v=%.1f (target %.1f)  err=%.2f"
                      % (run, o["x"], o["y"], o["yaw"], o["speed"], o["desired"], o["error"]),
                      flush=True)
                rows.append({"run": run, **o})
                with open(os.path.join(args.log_dir, "trace_%d.csv" % run), "w",
                          newline="") as handle:
                    w = csv.writer(handle)
                    w.writerow(["t", "x", "y", "speed", "desired", "error",
                                "throttle", "steering"])
                    w.writerows(watcher.trace)
            else:
                print("run %d: clean (max err %.2f)" % (run, watcher.error), flush=True)
                rows.append({"run": run, "x": float("nan"), "y": float("nan"),
                             "yaw": float("nan"), "speed": float("nan"),
                             "desired": float("nan"), "error": float("nan")})
    finally:
        watcher.destroy_node()
        rclpy.shutdown()
        kill_all()

    with open(args.out, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["run", "x", "y", "yaw", "speed",
                                                    "desired", "error"])
        writer.writeheader()
        writer.writerows(rows)
    print("Wrote " + args.out)


if __name__ == "__main__":
    main()
