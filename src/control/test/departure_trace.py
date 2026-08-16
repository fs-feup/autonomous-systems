# pylint: skip-file
# mypy: ignore-errors
"""Records what the car and controller were doing as a run leaves the track.

Every failure in the campaign is the same one: the car departs and does not recover, and no cost
weight or constraint bound changed the ~19% clean rate. Aggregate statistics cannot say WHY -
they only say that it happens - and three mechanisms inferred from aggregates this session turned
out to be wrong. This records the actual approach to a departure so the cause can be read rather
than guessed.

What it captures, at message rate, in a ring buffer around the onset:
  - tracking error and speed (ControlStatistics)
  - commanded throttle and steering (ControlCommand), to see whether the controller is asking for
    the recovery and not getting it, or not asking at all
  - driven-wheel slip ratio (WheelScalars), to separate "cannot recover" from "spinning"
  - acados t_tot and SQP iteration count, to see whether solves degrade before or after the car
    starts leaving - which settles cause vs symptom for the solver failures

The ring buffer matters: the interesting window is the two seconds BEFORE the error crosses the
threshold, which a latch-on-trigger recorder throws away.

Usage: python3 departure_trace.py --runs 4 [--threshold 2.0]
"""
import argparse
import collections
import csv
import os
import signal
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from custom_interfaces.msg import ControlCommand, ControlStatistics, WheelScalars


class Tracer(Node):
    def __init__(self, threshold, pre_samples):
        super().__init__("departure_tracer")
        self.threshold = threshold
        self.ring = collections.deque(maxlen=pre_samples)
        self.post = []
        self.onset_time = None
        self.error = 0.0
        self.speed = 0.0
        self.throttle = float("nan")
        self.steering = float("nan")
        self.slip = float("nan")
        self.t_tot = float("nan")
        self.sqp_iter = float("nan")
        self.t0 = time.time()
        self.create_subscription(ControlStatistics,
                                 "/invictasim/statistics/control_statistics", self._stats, 10)
        self.create_subscription(ControlCommand, "/control/command", self._cmd, 10)
        self.create_subscription(WheelScalars, "/invictasim/vehicle_model/tire/slip_ratio",
                                 self._slip, 10)
        self.create_subscription(Float64MultiArray, "/acados/execution_times", self._exec, 50)

    def _cmd(self, msg):
        self.throttle = msg.throttle_rl
        self.steering = msg.steering

    def _slip(self, msg):
        self.slip = max(abs(msg.rl), abs(msg.rr))

    def _exec(self, msg):
        if len(msg.data) > 5:
            self.t_tot = msg.data[0]
            self.sqp_iter = msg.data[5]

    def _stats(self, msg):
        self.error = abs(msg.tracking_error)
        self.speed = msg.current_velocity
        row = (round(time.time() - self.t0, 3), round(self.error, 3), round(self.speed, 2),
               round(self.throttle, 3), round(self.steering, 4), round(self.slip, 3),
               round(self.t_tot, 2), self.sqp_iter)
        if self.onset_time is None:
            self.ring.append(row)
            # Onset is the FIRST crossing, and the ring already holds what led up to it.
            if self.error > self.threshold and self.speed > 2.0:
                self.onset_time = time.time()
        else:
            self.post.append(row)


def spawn(cmd, path, env=None):
    log = open(path, "w")
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, preexec_fn=os.setsid,
                            env=env or os.environ.copy()), log


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=4)
    parser.add_argument("--threshold", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=70.0)
    parser.add_argument("--pre", type=int, default=120, help="samples kept before onset")
    parser.add_argument("--post", type=float, default=3.0, help="seconds kept after onset")
    parser.add_argument("--out-dir", default="/home/ws/src/control/test/mpcc_data/departures")
    parser.add_argument("--log-dir", default="/tmp/departure_logs")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)

    for run in range(args.runs):
        for pattern in ("lib/invictasim/invictasim", "ros2 launch invictasim",
                        "lib/planning/planning"):
            subprocess.run(["pkill", "-9", "-f", pattern],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-x", "node_control"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2.0)

        env = os.environ.copy()
        env["SDL_VIDEODRIVER"] = "dummy"
        sim, sl = spawn(["ros2", "launch", "invictasim", "invictasim.launch.py"],
                        os.path.join(args.log_dir, "sim.log"), env)
        time.sleep(14.0)
        plan, pl = spawn(["ros2", "run", "planning", "planning"],
                         os.path.join(args.log_dir, "plan_%d.log" % run))
        time.sleep(3.0)
        ctrl, cl = spawn(["ros2", "run", "control", "node_control"],
                         os.path.join(args.log_dir, "ctrl_%d.log" % run))

        # Fresh context per run: the global one cannot be re-inited after shutdown, and one
        # shared across runs goes invalid part-way through a multi-run session.
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = _make_tracer(context, args.threshold, args.pre)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        deadline = time.time() + args.timeout
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.2)
            if node.onset_time and time.time() - node.onset_time > args.post:
                break

        rows = list(node.ring) + node.post
        onset_marked = len(node.ring)
        for proc in (ctrl, plan, sim):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        for h in (cl, pl, sl):
            h.close()
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown(context=context)
        except Exception:
            pass

        if node.onset_time is None:
            print("run %d: no departure (max error %.2f)" % (run, node.error), flush=True)
            continue
        path = os.path.join(args.out_dir, "departure_%d.csv" % run)
        with open(path, "w", newline="") as handle:
            w = csv.writer(handle)
            w.writerow(["t", "error", "speed", "throttle", "steering", "slip", "t_tot",
                        "sqp_iter", "phase"])
            for i, row in enumerate(rows):
                w.writerow(list(row) + ["pre" if i < onset_marked else "post"])
        print("run %d: departure traced -> %s (%d pre, %d post)"
              % (run, path, onset_marked, len(node.post)), flush=True)


def _make_tracer(context, threshold, pre):
    """Tracer bound to an explicit context (the global one cannot be re-inited per run)."""
    class _T(Tracer):
        def __init__(self):
            Node.__init__(self, "departure_tracer", context=context)
            self.threshold = threshold
            self.ring = collections.deque(maxlen=pre)
            self.post = []
            self.onset_time = None
            self.error = 0.0
            self.speed = 0.0
            self.throttle = float("nan")
            self.steering = float("nan")
            self.slip = float("nan")
            self.t_tot = float("nan")
            self.sqp_iter = float("nan")
            self.t0 = time.time()
            self.create_subscription(ControlStatistics,
                                     "/invictasim/statistics/control_statistics", self._stats, 10)
            self.create_subscription(ControlCommand, "/control/command", self._cmd, 10)
            self.create_subscription(WheelScalars,
                                     "/invictasim/vehicle_model/tire/slip_ratio", self._slip, 10)
            self.create_subscription(Float64MultiArray, "/acados/execution_times", self._exec, 50)
    return _T()


if __name__ == "__main__":
    main()
