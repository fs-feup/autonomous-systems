#!/usr/bin/env python3
import os
import sys
import time

try:
    import yaml

    HAS_YAML = True
except ImportError:
    HAS_YAML = False
import subprocess
import signal
import pickle
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import LapSummary

from skopt import gp_minimize
from skopt.space import Real
from skopt.callbacks import CheckpointSaver

CONFIG_PATH = "config/control/invictasim.yaml"
OUTPUT_PARAMS_PATH = "config/control/best_pid_params.yaml"
CHECKPOINT_PATH = "config/control/bayesopt_checkpoint.pkl"


class LapSummarySubscriber(Node):
    def __init__(self):
        super().__init__("lap_summary_subscriber_tuner")
        self.latest_msg = None
        self.subscription = self.create_subscription(
            LapSummary, "invictasim/statistics/lap_summary", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        self.latest_msg = msg


def load_yaml(path):
    if HAS_YAML:
        with open(path, "r") as f:
            return yaml.safe_load(f)

    # Custom parser fallback for simple ROS config yaml
    data = {}
    current_key = None
    with open(path, "r") as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            if line.startswith("  ") or line.startswith("\t"):
                if ":" in stripped:
                    k, v = stripped.split(":", 1)
                    k = k.strip()
                    v = v.strip()
                    if " #" in v:
                        v = v.split(" #", 1)[0].strip()
                    try:
                        if "." in v:
                            v = float(v)
                        else:
                            v = int(v)
                    except ValueError:
                        if v.lower() == "true":
                            v = True
                        elif v.lower() == "false":
                            v = False
                        else:
                            v = v.strip("\"'")
                    if current_key:
                        data[current_key][k] = v
            else:
                if ":" in stripped:
                    k = stripped.split(":", 1)[0].strip()
                    data[k] = {}
                    current_key = k
    return data


def save_yaml(path, data):
    if HAS_YAML:
        with open(path, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=False)
        return

    # Custom writer fallback for simple ROS config yaml
    with open(path, "w") as f:
        for parent_key, child_dict in data.items():
            f.write(f"{parent_key}:\n")
            for k, v in child_dict.items():
                if isinstance(v, bool):
                    v_str = str(v).lower()
                else:
                    v_str = str(v)
                f.write(f"  {k}: {v_str}\n")


def kill_process_group(proc):
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
        for _ in range(10):
            if proc.poll() is not None:
                return
            time.sleep(0.5)
        os.killpg(pgid, signal.SIGTERM)
        for _ in range(6):
            if proc.poll() is not None:
                return
            time.sleep(0.5)
        os.killpg(pgid, signal.SIGKILL)
    except Exception:
        pass


def clean_up_all():
    subprocess.run(
        ["pkill", "-f", "install/invictasim/lib/invictasim/invictasim"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    subprocess.run(
        ["pkill", "-f", "install/control/lib/control/node_control"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    subprocess.run(
        ["pkill", "-f", "install/planning/lib/planning/planning"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(1)


# Cache to store evaluated parameters to avoid repeating simulations
eval_cache = {}

# Large, clearly-worse-than-any-real-run penalty for crashed/timed-out sims.
# Kept far from the range of real objective values so the GP treats
# these as "broken", not just "bad", and doesn't waste samples nearby.
FAILURE_OBJ = 1e4


def run_simulation(node, kp, ki, kd, max_err, repeat_index=0):
    # Check cache first. repeat_index is folded into the key so that
    # deliberately repeated evaluations of the same params (see
    # evaluate_candidate) each launch a fresh sim instead of being
    # short-circuited by the cache.
    param_key = (
        round(kp, 4),
        round(ki, 5),
        round(kd, 4),
        round(max_err, 2),
        repeat_index,
    )
    if param_key in eval_cache:
        print(f"  [Cache Hit] Reusing results for parameters: {param_key}")
        return eval_cache[param_key]

    print(
        f"\n--- Running evaluation with kp={kp:.3f}, ki={ki:.4f}, kd={kd:.3f}, max_err={max_err:.1f} ---"
    )

    # 1. Update YAML config
    config = load_yaml(CONFIG_PATH)
    config["control"]["pid_kp"] = float(kp)
    config["control"]["pid_ki"] = float(ki)
    config["control"]["pid_kd"] = float(kd)
    config["control"]["pid_max_positive_error"] = float(max_err)
    config["control"]["pid_max_negative_error"] = float(-max_err)
    save_yaml(CONFIG_PATH, config)

    # Make sure no leftover nodes are running
    clean_up_all()

    # Reset subscriber node's message
    node.latest_msg = None

    cmd_prefix = "source /opt/ros/humble/setup.bash && source install/setup.bash && "

    sim_proc = None
    control_proc = None
    planning_proc = None

    success = False
    stats = None

    try:
        sim_proc = subprocess.Popen(
            ["bash", "-c", cmd_prefix + "ros2 launch invictasim invictasim.launch.py"],
            preexec_fn=os.setsid,
        )
        time.sleep(4)
        control_proc = subprocess.Popen(
            ["bash", "-c", cmd_prefix + "ros2 launch control control.launch.py"],
            preexec_fn=os.setsid,
        )
        time.sleep(4)
        planning_proc = subprocess.Popen(
            ["bash", "-c", cmd_prefix + "ros2 launch planning planning.launch.py"],
            preexec_fn=os.setsid,
        )

        start_time = time.time()

        # 120s timeout
        while rclpy.ok() and (time.time() - start_time) < 120:
            rclpy.spin_once(node, timeout_sec=0.2)

            # Check if planning node or control node died (e.g. MPC crash)
            if control_proc.poll() is not None or planning_proc.poll() is not None:
                print("  [ERROR] A controller or planning node died prematurely.")
                break

            if node.latest_msg is not None and len(node.latest_msg.rows) >= 1:
                lap_1_row = node.latest_msg.rows[0]
                stats = {
                    "time": lap_1_row.time,
                    "cones_hit": lap_1_row.cones_hit,
                    "avg_velocity": lap_1_row.avg_velocity,
                    "avg_tracking_error_distance": lap_1_row.avg_tracking_error_distance,
                    "avg_velocity_error": lap_1_row.avg_velocity_error,
                    "max_velocity_error": lap_1_row.max_velocity_error,
                }
                success = True
                break
    except Exception as e:
        print(f"  [ERROR] Exception during run: {e}")
    finally:
        kill_process_group(planning_proc)
        kill_process_group(control_proc)
        kill_process_group(sim_proc)
        clean_up_all()

    # Calculate objective value
    if success and stats:
        # Velocity-only objective. Path tracking error is dropped entirely --
        # it isn't what this PID is responsible for. cones_hit stays in as
        # a hard safety penalty; time stays in so a "safe but slow" velocity
        # profile isn't free.
        obj_val = (
            stats["avg_velocity_error"] * 3.0
            + stats["time"] * 0.5
        )
        print(
            f"  [SUCCESS] Lap 1 Time: {stats['time']:.2f}s | Cones: {stats['cones_hit']} | Avg Tracking Err: {stats['avg_tracking_error_distance']:.4f}m | Avg Vel Err: {stats['avg_velocity_error']:.2f} km/h"
        )
        print(f"  --> Calculated Objective: {obj_val:.4f}")
    else:
        obj_val = FAILURE_OBJ
        print("  [FAILURE] Simulation run timed out or crashed.")
        print(f"  --> Calculated Objective: {obj_val:.4f}")
        stats = {}

    eval_cache[param_key] = (obj_val, stats)
    return obj_val, stats


REPEATS_PER_CANDIDATE = 2  # bump to 3 if you have sim budget to spare


def evaluate_candidate(node, kp, ki, kd, max_err, repeats=REPEATS_PER_CANDIDATE):
    """
    Run `repeats` independent sims for the same parameter set and return
    the median objective, plus the individual per-repeat stats.

    We use the median rather than the mean because a single crashed/timed
    out repeat (objective = FAILURE_OBJ) would otherwise dominate a mean
    and make an otherwise-good candidate look terrible; median is more
    robust to that kind of outlier while still penalizing candidates that
    are unreliable more than once.
    """
    objs = []
    all_stats = []
    for i in range(repeats):
        obj_val, stats = run_simulation(node, kp, ki, kd, max_err, repeat_index=i)
        objs.append(obj_val)
        all_stats.append(stats)
    objs_sorted = sorted(objs)
    median_obj = objs_sorted[len(objs_sorted) // 2]
    if repeats > 1:
        print(
            f"  [REPEATS] Objectives across {repeats} runs: {[round(o, 2) for o in objs]} -> median {median_obj:.4f}"
        )
    return median_obj, all_stats


# Global values for clean exit
original_config = None
best_params = None
best_obj_global = None


def signal_handler(sig, frame):
    print("\n\n[INTERRUPTED] Exiting tuning script early...")
    if original_config:
        save_yaml(CONFIG_PATH, original_config)
        print("Restored original control configuration.")
    # CheckpointSaver already writes the latest state to disk after every
    # completed iteration, so there's nothing extra to save here -- just
    # confirm it's present.
    if os.path.exists(CHECKPOINT_PATH):
        print(f"Latest Bayesian optimization state is on disk at {CHECKPOINT_PATH}")
    if best_params:
        print(f"\n======================================")
        print(f"BEST PARAMETERS FOUND SO FAR:")
        print(f"  kp: {best_params[0]:.3f}")
        print(f"  ki: {best_params[1]:.4f}")
        print(f"  kd: {best_params[2]:.3f}")
        print(f"  max_err: {best_params[3]:.1f}")
        print(f"======================================")
    sys.exit(0)


def main():
    global original_config, best_params, best_obj_global
    signal.signal(signal.SIGINT, signal_handler)

    # Back up the original config
    original_config = load_yaml(CONFIG_PATH)
    print("Backed up original control config.")

    rclpy.init()
    node = LapSummarySubscriber()

    # Seeded from the best point found in the previous tuning run, PLUS
    # a couple of explicit points with meaningfully nonzero ki. Since ki
    # never got picked in the last run (partly due to a tight max_err
    # clamp starving it, partly because it was never that consequential
    # to a tracking-error-dominated objective), we force the optimizer to
    # actually evaluate what integral action does rather than relying on
    # it to discover that on its own.
    x0_points = [
        [0.10, 0.00, 0.002, 1.5],  # previous best (max_err bumped to fit new bound)
        [0.10, 0.02, 0.002, 3.0],  # same gains, meaningful ki forced in
        [0.20, 0.01, 0.01, 3.0],  # a different region, also with ki > 0
    ]

    # Search space
    # - ki upper bound raised: 0.02 may have been too small to show any
    #   real effect once the objective was dominated by tracking error.
    # - max_err lower bound raised from 1.0: max_err clamps the error
    #   BEFORE it's integrated (anti-windup), so a very tight clamp can
    #   starve ki of anything to accumulate even if ki > 0. Giving it more
    #   room lets ki actually do something if it's useful.
    space = [
        Real(0.1, 0.8, name="kp"),
        Real(0.0, 0.05, name="ki"),
        Real(0.0, 0.15, name="kd"),
        Real(1.5, 10.0, name="max_err"),
    ]

    def objective(params):
        kp, ki, kd, max_err = params
        obj_val, all_stats = evaluate_candidate(node, kp, ki, kd, max_err)

        # Track running best so signal_handler / final printout have it
        global best_params, best_obj_global
        if best_obj_global is None or obj_val < best_obj_global:
            best_obj_global = obj_val
            best_params = (kp, ki, kd, max_err)
            if obj_val < FAILURE_OBJ:
                save_yaml(
                    OUTPUT_PARAMS_PATH,
                    {
                        "control": {
                            "pid_kp": float(kp),
                            "pid_ki": float(ki),
                            "pid_kd": float(kd),
                            "pid_max_positive_error": float(max_err),
                            "pid_max_negative_error": float(-max_err),
                        }
                    },
                )
        return obj_val

    # store_objective=False: the objective is a closure defined inside main()
    # and can't be pickled. We don't need it saved to resume later anyway --
    # we just pass the same objective function back in manually if resuming.
    checkpoint_saver = CheckpointSaver(
        CHECKPOINT_PATH, compress=3, store_objective=False
    )

    print("\nStarting Bayesian optimization (gp_minimize)...")
    print(
        f"Seeding search with {len(x0_points)} explicit points (incl. forced nonzero-ki samples):"
    )
    for pt in x0_points:
        print(f"  kp={pt[0]}, ki={pt[1]}, kd={pt[2]}, max_err={pt[3]}")

    # NOTE: each gp_minimize "call" now triggers REPEATS_PER_CANDIDATE sims
    # (see evaluate_candidate), so total sims run = N_CALLS * REPEATS_PER_CANDIDATE.
    # 15 * 2 = 30 sims, roughly the same total budget as the last run's 25.
    N_CALLS = 15  # total candidates to evaluate
    N_INITIAL_POINTS = 5  # random exploration before the GP model kicks in

    result = gp_minimize(
        objective,
        space,
        x0=x0_points,
        n_calls=N_CALLS,
        n_initial_points=N_INITIAL_POINTS,
        noise="gaussian",  # sim results are noisy run-to-run; don't overfit to single runs
        random_state=42,
        callback=[checkpoint_saver],
    )

    best_x = result.x
    best_obj = result.fun
    best_params = tuple(best_x)

    print(f"\n======================================")
    print(f"TUNING COMPLETED!")
    print(f"Best Objective: {best_obj:.4f}")
    print(f"Best Parameters:")
    print(f"  kp: {best_params[0]:.3f}")
    print(f"  ki: {best_params[1]:.4f}")
    print(f"  kd: {best_params[2]:.3f}")
    print(f"  max_err: {best_params[3]:.1f}")

    # Look up cached stats for the best point (all repeats), if available
    kp_b, ki_b, kd_b, max_err_b = best_params
    base_key = (round(kp_b, 4), round(ki_b, 5), round(kd_b, 4), round(max_err_b, 2))
    repeat_stats = []
    for i in range(REPEATS_PER_CANDIDATE):
        cached = eval_cache.get(base_key + (i,))
        if cached and cached[1]:
            repeat_stats.append(cached[1])
    if repeat_stats:
        print(f"Best Stats on Lap 1 (across {len(repeat_stats)} repeat(s)):")
        for i, s in enumerate(repeat_stats):
            print(
                f"  Repeat {i}: Time={s['time']:.2f}s | Cones={s['cones_hit']} | "
                f"AvgTrackErr={s['avg_tracking_error_distance']:.4f}m | AvgVelErr={s['avg_velocity_error']:.2f} km/h"
            )
    print(f"======================================")

    # Restore original config
    save_yaml(CONFIG_PATH, original_config)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
