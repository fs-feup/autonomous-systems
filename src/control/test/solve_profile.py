# pylint: skip-file
# mypy: ignore-errors
"""Profiles acados solve time over a run, split by phase.

A single mean is misleading here: the sweep reported ~14 ms for a controller that reads ~3 ms in
a terminal, and the gap is most likely that the mean folds in a slow startup transient (cold
caches, no warm start, standing-start transients) that a human watching steady-state numbers
never sees. This prints the distribution and splits early from settled so the two readings can be
compared directly rather than argued about.

Usage: python3 solve_profile.py [--controller supermpc] [--seconds 45]
"""
import argparse
import os
import signal
import statistics as st
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class Profiler(Node):
    def __init__(self):
        super().__init__("solve_profiler")
        self.samples = []  # (t_wall, t_tot_ms)
        self.full = []     # whole array, to compare against what a dashboard might plot
        self.t0 = time.time()
        self.create_subscription(Float64MultiArray, "/acados/execution_times", self._cb, 50)

    def _cb(self, msg):
        if msg.data:
            self.samples.append((time.time() - self.t0, msg.data[0]))
            self.full.append(list(msg.data))


def spawn(cmd, path, env=None):
    log = open(path, "w")
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, preexec_fn=os.setsid,
                            env=env or os.environ.copy()), log


def pct(values, q):
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=45.0)
    parser.add_argument("--sdl", default="dummy")
    parser.add_argument("--log-dir", default="/tmp/solve_profile")
    args = parser.parse_args()
    os.makedirs(args.log_dir, exist_ok=True)

    for pattern in ("lib/invictasim/invictasim", "ros2 launch invictasim",
                    "lib/planning/planning"):
        subprocess.run(["pkill", "-9", "-f", pattern],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-x", "node_control"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    env = os.environ.copy()
    env["SDL_VIDEODRIVER"] = args.sdl
    sim, sl = spawn(["ros2", "launch", "invictasim", "invictasim.launch.py"],
                    os.path.join(args.log_dir, "sim.log"), env)
    time.sleep(14.0)
    plan, pl = spawn(["ros2", "run", "planning", "planning"],
                     os.path.join(args.log_dir, "plan.log"))
    time.sleep(3.0)
    ctrl, cl = spawn(["ros2", "run", "control", "node_control"],
                     os.path.join(args.log_dir, "ctrl.log"))

    rclpy.init()
    node = Profiler()
    deadline = time.time() + args.seconds
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        for proc in (ctrl, plan, sim):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        for h in (cl, pl, sl):
            h.close()
        node.destroy_node()
        rclpy.shutdown()

    samples = node.samples
    if not samples:
        print("no samples on /acados/execution_times")
        return
    times = [v for _, v in samples]
    print("samples: %d over %.1f s" % (len(times), samples[-1][0] - samples[0][0]))
    print("  overall  mean %6.2f  median %6.2f  p90 %6.2f  p99 %6.2f  max %6.2f"
          % (st.mean(times), st.median(times), pct(times, 0.90), pct(times, 0.99), max(times)))
    for lo, hi, label in ((0, 5, "first 5 s"), (5, 15, "5-15 s"), (15, 1e9, "settled")):
        window = [v for t, v in samples if lo <= t < hi]
        if window:
            print("  %-9s mean %6.2f  median %6.2f  p90 %6.2f  max %6.2f  (n=%d)"
                  % (label, st.mean(window), st.median(window),
                     pct(window, 0.90), max(window), len(window)))
    # The published array is [t_tot, t_lin, t_sim, t_qp, t_reg, sqp_iter, avg_lin, avg_qp,
    # avg_reg]. Index 0 is the WHOLE solve; the others are components and are naturally much
    # smaller, so a dashboard plotting the wrong element shows a reassuring number that is not
    # the thing bounded by the control period.
    labels = ["t_tot", "t_lin", "t_sim", "t_qp", "t_reg", "sqp_iter",
              "avg_lin", "avg_qp", "avg_reg"]
    if node.full:
        width = min(len(labels), min(len(r) for r in node.full))
        print("  breakdown (median over run):")
        for i in range(width):
            column = sorted(r[i] for r in node.full)
            print("    [%d] %-9s %8.3f" % (i, labels[i], column[len(column) // 2]))
    over = sum(1 for v in times if v > 25.0)
    print("  solves over the 25 ms command period: %d (%.1f%%)" % (over, 100.0 * over / len(times)))


if __name__ == "__main__":
    main()
