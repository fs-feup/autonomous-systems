#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
from pathlib import Path
import random
import shlex
import shutil
import signal
import subprocess
import time

import yaml


WORKSPACE = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = WORKSPACE / "config/invictasim/tuning/02_vehicle_tuning.yaml"
GLOBAL_CONFIG = WORKSPACE / "config/invictasim/global.yaml"
ROS_LOG_DIR = WORKSPACE / "log/invictasim_tuning_ros"


def load_yaml(path):
    with Path(path).open("r") as handle:
        return yaml.safe_load(handle)


def dump_yaml(path, data):
    with Path(path).open("w") as handle:
        yaml.safe_dump(data, handle, sort_keys=False)


def get_nested(data, keys):
    node = data
    for key in keys:
        node = node[key]
    return node


def set_nested(data, keys, value):
    node = data
    for key in keys[:-1]:
        node = node[key]
    node[keys[-1]] = float(value)


def set_nested_raw(data, keys, value):
    node = data
    for key in keys[:-1]:
        node = node[key]
    node[keys[-1]] = value


def ros_command(command):
    ROS_LOG_DIR.mkdir(parents=True, exist_ok=True)
    return (
        f"export ROS_LOG_DIR={shlex.quote(str(ROS_LOG_DIR))} && "
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(WORKSPACE / 'install/setup.bash'))} && "
        f"{command}"
    )


def start_process(command):
    process = subprocess.Popen(
        ["bash", "-lc", ros_command(command)],
        cwd=WORKSPACE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    return process


def stop_process(process, grace_s=3.0):
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=grace_s)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            try:
                process.wait(timeout=grace_s)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait()


def restore_files(original_text):
    for path, text in original_text.items():
        path.write_text(text)


def apply_values(parameters, values):
    by_file = {}
    for spec, value in zip(parameters, values):
        path = WORKSPACE / spec["file"]
        by_file.setdefault(path, load_yaml(path))
        set_nested(by_file[path], spec["path"], value)
    for path, data in by_file.items():
        dump_yaml(path, data)


def apply_invictasim_runtime_config(bag, playback):
    original_text = GLOBAL_CONFIG.read_text()
    data = load_yaml(GLOBAL_CONFIG)
    changed = False
    if bag.get("track_name"):
        set_nested_raw(data, ["invictasim", "track_name"], str(bag["track_name"]))
        changed = True
    sim_speed = playback.get("sim_speed")
    if sim_speed is not None:
        set_nested_raw(data, ["invictasim", "sim_speed"], float(sim_speed))
        changed = True
    if changed:
        dump_yaml(GLOBAL_CONFIG, data)
        return original_text
    return None


def remove_path_if_empty(path):
    path = Path(path)
    while path != WORKSPACE and path.exists():
        try:
            path.rmdir()
        except OSError:
            return
        path = path.parent


def read_candidate_csv(csv_path):
    rows = []
    with csv_path.open("r", newline="") as handle:
        for row in csv.DictReader(handle):
            rows.append(row)
    return rows


def rmse(values):
    values = [value for value in values if value is not None and math.isfinite(value)]
    if not values:
        return None
    return math.sqrt(sum(value * value for value in values) / len(values))


def as_float(row, key):
    value = row.get(key, "")
    if value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def score_csv(csv_path, score_config):
    rows = read_candidate_csv(csv_path)
    if len(rows) < int(score_config.get("min_samples", 0)):
        return {
            "score": float("inf"),
            "samples": len(rows),
            "error": f"not enough samples in {csv_path}",
        }

    position_rmse = rmse([as_float(row, "position_error") for row in rows])
    heading_rmse = rmse([as_float(row, "heading_error") for row in rows])

    velocity_errors = []
    yaw_rate_errors = []
    for row in rows:
        rvx = as_float(row, "real_velocity_x")
        rvy = as_float(row, "real_velocity_y")
        svx = as_float(row, "sim_velocity_x")
        svy = as_float(row, "sim_velocity_y")
        ryr = as_float(row, "real_yaw_rate")
        syr = as_float(row, "sim_yaw_rate")
        if None not in (rvx, rvy, svx, svy):
            velocity_errors.append(math.hypot(svx - rvx, svy - rvy))
        if None not in (ryr, syr):
            yaw_rate_errors.append(syr - ryr)

    velocity_rmse = rmse(velocity_errors)
    yaw_rate_rmse = rmse(yaw_rate_errors)
    front_wheel_rpm_rmse = rmse([as_float(row, "front_wheel_rpm_error") for row in rows])
    motor_rpm_rmse = rmse([as_float(row, "motor_rpm_error") for row in rows])

    metrics = {
        "position_rmse": position_rmse,
        "heading_rmse": heading_rmse,
        "velocity_rmse": velocity_rmse,
        "yaw_rate_rmse": yaw_rate_rmse,
        "front_wheel_rpm_rmse": front_wheel_rpm_rmse,
        "motor_rpm_rmse": motor_rpm_rmse,
        "samples": len(rows),
    }
    weights = score_config.get("weights", {})
    score = 0.0
    for key, weight in weights.items():
        value = metrics.get(key)
        if value is None:
            return {"score": float("inf"), **metrics, "error": f"missing {key}"}
        score += float(weight) * value
    metrics["score"] = score
    return metrics


def newest_csv(directory):
    files = sorted(Path(directory).glob("tuning_*.csv"), key=lambda path: path.stat().st_mtime)
    return files[-1] if files else None


def append_status(candidate_dir, message):
    timestamp = time.strftime("%H:%M:%S")
    with (candidate_dir / "status.txt").open("a") as handle:
        handle.write(f"[{timestamp}] {message}\n")


def run_bag_trial(config, candidate_dir, bag, trial_name):
    playback = config["tuning"]["playback"]
    candidate_dir.mkdir(parents=True, exist_ok=True)

    csv_dir = candidate_dir / "evaluator_csv" / trial_name
    csv_dir.mkdir(parents=True, exist_ok=True)

    sim_process = None
    bag_process = None
    original_global_config = None
    try:
        original_global_config = apply_invictasim_runtime_config(bag, playback)
        if bag.get("track_name"):
            append_status(candidate_dir, f"{trial_name}: using track {bag['track_name']}")
        if playback.get("sim_speed") is not None:
            append_status(candidate_dir, f"{trial_name}: using sim_speed {playback['sim_speed']}")
            if float(playback.get("rate", playback["sim_speed"])) != float(playback["sim_speed"]):
                append_status(
                    candidate_dir,
                    f"{trial_name}: WARNING bag rate {playback.get('rate')} differs from sim_speed "
                    f"{playback['sim_speed']}",
                )

        track_label = f" with track {bag['track_name']}" if bag.get("track_name") else ""
        print(f"[{trial_name}] starting InvictaSim{track_label}", flush=True)
        append_status(candidate_dir, f"{trial_name}: starting InvictaSim{track_label}")
        sim_cmd = (
            "ros2 run invictasim invictasim --ros-args "
            f"-p tuning_output_directory:={shlex.quote(str(csv_dir))}"
        )
        sim_process = start_process(sim_cmd)
        append_status(candidate_dir, f"{trial_name}: InvictaSim pid {sim_process.pid}")
        time.sleep(float(playback.get("startup_sleep_s", 3.0)))
        if sim_process.poll() is not None:
            append_status(candidate_dir, f"{trial_name}: InvictaSim exited early")
            return {
                "score": float("inf"),
                "error": f"InvictaSim exited early with code {sim_process.returncode}",
            }

        topics = " ".join(shlex.quote(topic) for topic in playback.get("topics", []))
        playback_rate = float(playback.get("rate", playback.get("sim_speed", 1.0)))
        bag_cmd = (
            f"ros2 bag play {shlex.quote(str(bag['path']))} "
            f"--rate {playback_rate} "
            "--disable-keyboard-controls"
        )
        if topics:
            bag_cmd += f" --topics {topics}"
        if float(bag.get("start_offset_s", 0.0)) > 0.0:
            bag_cmd += f" --start-offset {float(bag['start_offset_s'])}"

        print(f"[{trial_name}] playing bag: {bag['path']}", flush=True)
        append_status(candidate_dir, f"{trial_name}: playing bag {bag['path']}")
        bag_process = subprocess.Popen(
            ["bash", "-lc", ros_command(bag_cmd)],
            cwd=WORKSPACE,
            stdout=None,
            stderr=None,
            preexec_fn=os.setsid,
        )
        append_status(candidate_dir, f"{trial_name}: bag pid {bag_process.pid}")
        bag_timeout = bag.get("max_wall_time_s")
        try:
            bag_returncode = bag_process.wait(
                timeout=float(bag_timeout) if bag_timeout else None
            )
        except subprocess.TimeoutExpired:
            append_status(candidate_dir, f"{trial_name}: bag timeout after {bag_timeout}s")
            stop_process(bag_process, float(playback.get("shutdown_grace_s", 3.0)))
            bag_returncode = 124

        append_status(candidate_dir, f"{trial_name}: bag exit code {bag_returncode}")
        if bag_returncode not in (0, 124):
            return {
                "score": float("inf"),
                "error": f"bag playback failed with exit code {bag_returncode}",
            }
        time.sleep(0.5)
    finally:
        grace = float(playback.get("shutdown_grace_s", 3.0))
        if bag_process is not None:
            stop_process(bag_process, grace)
        if sim_process is not None:
            stop_process(sim_process, grace)
        if original_global_config is not None:
            GLOBAL_CONFIG.write_text(original_global_config)

    csv_path = newest_csv(csv_dir)
    if csv_path is None:
        append_status(candidate_dir, f"{trial_name}: evaluator did not produce csv")
        remove_path_if_empty(csv_dir)
        return {"score": float("inf"), "error": "evaluator did not produce csv"}
    result = score_csv(csv_path, config["tuning"]["score"])
    if bag.get("track_name"):
        result["track_name"] = str(bag["track_name"])
    try:
        csv_path.unlink()
        remove_path_if_empty(csv_path.parent)
    except OSError as exc:
        append_status(candidate_dir, f"{trial_name}: could not remove evaluator csv: {exc}")
    append_status(candidate_dir, f"{trial_name}: score {result['score']}")
    return result


def run_candidate(config, parameters, values, run_dir, candidate_index):
    apply_values(parameters, values)
    candidate_dir = run_dir / "tries" / f"try_{candidate_index:04d}"
    bag_results = []
    weighted_score = 0.0
    total_weight = 0.0
    for bag_index, bag in enumerate(config["tuning"]["bags"]):
        result = run_bag_trial(config, candidate_dir, bag, f"bag_{bag_index:02d}")
        bag_results.append(result)
        weight = float(bag.get("weight", 1.0))
        weighted_score += weight * result["score"]
        total_weight += weight
    score = weighted_score / max(total_weight, 1e-9)
    return {
        "try": candidate_index,
        "score": score,
        "values": {spec["name"]: value for spec, value in zip(parameters, values)},
        "bags": bag_results,
    }


def baseline_values(parameters):
    values = []
    for spec in parameters:
        values.append(float(get_nested(load_yaml(WORKSPACE / spec["file"]), spec["path"])))
    return values


def random_values(parameters, rng):
    return [rng.uniform(float(spec["min"]), float(spec["max"])) for spec in parameters]


def flatten_result(result):
    row = {
        "try": result["try"],
        "kind": result.get("kind", ""),
        "score": result["score"],
    }
    for name, value in result["values"].items():
        row[f"param.{name}"] = value
    if result["bags"]:
        first_bag = result["bags"][0]
        for key in (
            "position_rmse",
            "heading_rmse",
            "velocity_rmse",
            "yaw_rate_rmse",
            "front_wheel_rpm_rmse",
            "motor_rpm_rmse",
            "samples",
            "error",
        ):
            row[key] = first_bag.get(key, "")
    return row


def write_result(run_dir, result):
    with (run_dir / "results.jsonl").open("a") as handle:
        handle.write(json.dumps(result, sort_keys=True) + "\n")
    try_dir = run_dir / "tries" / f"try_{result['try']:04d}"
    try_dir.mkdir(parents=True, exist_ok=True)
    (try_dir / "result.json").write_text(json.dumps(result, indent=2))

    row = flatten_result(result)
    summary_path = run_dir / "summary.csv"
    write_header = not summary_path.exists()
    with summary_path.open("a", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def copy_best_csvs(run_dir, best_result):
    best_dir = run_dir / "best"
    best_dir.mkdir(exist_ok=True)
    (best_dir / "best_parameters.json").write_text(json.dumps(best_result, indent=2))


def run_random(config, parameters, run_dir):
    optimizer = config["tuning"].get("optimizer", {})
    rng = random.Random(int(optimizer.get("seed", 7)))
    trials = int(optimizer.get("trials", 24))

    best = None
    candidate_index = 0

    values = baseline_values(parameters)
    result = run_candidate(config, parameters, values, run_dir, candidate_index)
    result["kind"] = "baseline"
    write_result(run_dir, result)
    best = result
    candidate_index += 1

    for _ in range(trials):
        values = random_values(parameters, rng)
        result = run_candidate(config, parameters, values, run_dir, candidate_index)
        result["kind"] = "random"
        write_result(run_dir, result)
        if result["score"] < best["score"]:
            best = result
            copy_best_csvs(run_dir, best)
        candidate_index += 1
    return best


def run_differential_evolution(config, parameters, run_dir):
    from scipy.optimize import differential_evolution

    optimizer = config["tuning"].get("optimizer", {})
    bounds = [(float(spec["min"]), float(spec["max"])) for spec in parameters]
    best = {"score": float("inf")}
    counter = {"value": 0}

    values = baseline_values(parameters)
    baseline = run_candidate(config, parameters, values, run_dir, counter["value"])
    baseline["kind"] = "baseline"
    write_result(run_dir, baseline)
    best = baseline
    counter["value"] += 1

    def objective(vector):
        nonlocal best
        result = run_candidate(config, parameters, list(vector), run_dir, counter["value"])
        result["kind"] = "differential_evolution"
        write_result(run_dir, result)
        if result["score"] < best["score"]:
            best = result
            copy_best_csvs(run_dir, best)
        counter["value"] += 1
        return result["score"]

    differential_evolution(
        objective,
        bounds,
        maxiter=int(optimizer.get("maxiter", 5)),
        popsize=int(optimizer.get("popsize", 4)),
        seed=int(optimizer.get("seed", 7)),
        polish=False,
        workers=1,
        updating="immediate",
    )
    return best


def main():
    parser = argparse.ArgumentParser(description="Tune InvictaSim vehicle parameters from rosbags.")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--run-dir", default="")
    parser.add_argument("--optimizer", choices=["random", "de"], default="")
    parser.add_argument("--trials", type=int, default=None)
    parser.add_argument(
        "--maxiter",
        type=int,
        default=None,
        help="Differential evolution iterations. Only used with --optimizer de.",
    )
    parser.add_argument(
        "--popsize",
        type=int,
        default=None,
        help="Differential evolution population multiplier. Only used with --optimizer de.",
    )
    parser.add_argument(
        "--bag-time",
        type=float,
        default=0.0,
        help="Override each bag max_wall_time_s for quick smoke tests.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.0,
        help="Run InvictaSim and rosbag playback faster by the same multiplier.",
    )
    parser.add_argument("--apply-best", action="store_true")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_yaml(config_path)
    parameters = config["tuning"]["parameters"]
    if args.optimizer:
        config["tuning"].setdefault("optimizer", {})["method"] = args.optimizer
    if args.trials is not None:
        config["tuning"].setdefault("optimizer", {})["trials"] = args.trials
    if args.maxiter is not None:
        config["tuning"].setdefault("optimizer", {})["maxiter"] = args.maxiter
    if args.popsize is not None:
        config["tuning"].setdefault("optimizer", {})["popsize"] = args.popsize
    if args.bag_time:
        for bag in config["tuning"]["bags"]:
            bag["max_wall_time_s"] = args.bag_time
    if args.speed:
        playback = config["tuning"].setdefault("playback", {})
        playback["sim_speed"] = args.speed
        playback["rate"] = args.speed

    run_dir = Path(args.run_dir) if args.run_dir else (
        WORKSPACE / "performance/invictasim_tuning" / time.strftime("tuning_%Y%m%d_%H%M%S")
    )
    run_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(config_path, run_dir / config_path.name)

    touched_files = {WORKSPACE / spec["file"] for spec in parameters}
    original_text = {path: path.read_text() for path in touched_files}

    best = None
    try:
        method = config["tuning"].get("optimizer", {}).get("method", "random")
        if method == "de":
            best = run_differential_evolution(config, parameters, run_dir)
        else:
            best = run_random(config, parameters, run_dir)
        copy_best_csvs(run_dir, best)
    finally:
        restore_files(original_text)

    if best and args.apply_best:
        apply_values(parameters, [best["values"][spec["name"]] for spec in parameters])

    print(json.dumps({"run_dir": str(run_dir), "best": best}, indent=2))


if __name__ == "__main__":
    main()
