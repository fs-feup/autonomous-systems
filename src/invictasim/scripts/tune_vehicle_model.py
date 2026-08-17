#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
from pathlib import Path
import random
import shlex
import signal
import subprocess
import time

import yaml


WORKSPACE = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = WORKSPACE / "config/invictasim/tuning/02_acceleration_longitudinal_tuning.yaml"
GLOBAL_CONFIG = WORKSPACE / "config/invictasim/global.yaml"
SHARED_GLOBAL_CONFIG = WORKSPACE / "config/global/global_config.yaml"
ROS_LOG_DIR = WORKSPACE / "log/invictasim_tuning_ros"
DEFAULT_TUNING_ROOT = WORKSPACE / "performance/invictasim_tuning"


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
    original_text = {}
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
        original_text[GLOBAL_CONFIG] = GLOBAL_CONFIG.read_text()
        dump_yaml(GLOBAL_CONFIG, data)

    if bag.get("discipline"):
        global_data = load_yaml(SHARED_GLOBAL_CONFIG)
        set_nested_raw(global_data, ["global", "discipline"], str(bag["discipline"]))
        original_text[SHARED_GLOBAL_CONFIG] = SHARED_GLOBAL_CONFIG.read_text()
        dump_yaml(SHARED_GLOBAL_CONFIG, global_data)

    return original_text or None


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


def mean(values):
    values = [value for value in values if value is not None and math.isfinite(value)]
    if not values:
        return None
    return sum(values) / len(values)


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
    start_s = score_config.get("start_s")
    end_s = score_config.get("end_s")
    if start_s is not None or end_s is not None:
        filtered_rows = []
        for row in rows:
            elapsed_s = as_float(row, "elapsed_s")
            if elapsed_s is None:
                continue
            if start_s is not None and elapsed_s < float(start_s):
                continue
            if end_s is not None and elapsed_s > float(end_s):
                continue
            filtered_rows.append(row)
        rows = filtered_rows

    def calculate_metrics(metric_rows, min_samples, min_usable_samples=0):
        if len(metric_rows) < int(min_samples):
            if len(metric_rows) < int(min_usable_samples):
                return {
                    "score": float("inf"),
                    "samples": len(metric_rows),
                    "error": f"not enough samples in {csv_path}",
                }

        position_rmse = rmse([as_float(row, "position_error") for row in metric_rows])
        heading_rmse = rmse([as_float(row, "heading_error") for row in metric_rows])

        velocity_errors = []
        velocity_x_errors = []
        velocity_x_slow_errors = []
        velocity_y_errors = []
        yaw_rate_errors = []
        yaw_rate_under_errors = []
        for row in metric_rows:
            rvx = as_float(row, "real_velocity_x")
            rvy = as_float(row, "real_velocity_y")
            svx = as_float(row, "sim_velocity_x")
            svy = as_float(row, "sim_velocity_y")
            ryr = as_float(row, "real_yaw_rate")
            syr = as_float(row, "sim_yaw_rate")
            if None not in (rvx, rvy, svx, svy):
                velocity_x_error = svx - rvx
                velocity_errors.append(math.hypot(velocity_x_error, svy - rvy))
                velocity_x_errors.append(velocity_x_error)
                velocity_x_slow_errors.append(min(0.0, velocity_x_error))
                velocity_y_errors.append(svy - rvy)
            if None not in (ryr, syr):
                yaw_rate_error = syr - ryr
                yaw_rate_errors.append(yaw_rate_error)
                yaw_rate_under_errors.append(min(0.0, yaw_rate_error))

        metrics = {
            "position_rmse": position_rmse,
            "heading_rmse": heading_rmse,
            "velocity_rmse": rmse(velocity_errors),
            "velocity_x_rmse": rmse(velocity_x_errors),
            "velocity_x_mean_error": mean(velocity_x_errors),
            "velocity_x_slow_rmse": rmse(velocity_x_slow_errors),
            "velocity_x_slow_mean": -mean(velocity_x_slow_errors)
            if mean(velocity_x_slow_errors) is not None
            else None,
            "velocity_y_rmse": rmse(velocity_y_errors),
            "yaw_rate_rmse": rmse(yaw_rate_errors),
            "yaw_rate_under_rmse": rmse(yaw_rate_under_errors),
            "front_wheel_rpm_rmse": rmse(
                [as_float(row, "front_wheel_rpm_error") for row in metric_rows]
            ),
            "motor_rpm_rmse": rmse([as_float(row, "motor_rpm_error") for row in metric_rows]),
            "samples": len(metric_rows),
        }
        return metrics

    metrics = calculate_metrics(
        rows,
        score_config.get("min_samples", 0),
        score_config.get("min_usable_samples", score_config.get("min_samples", 0)),
    )
    if metrics.get("score") == float("inf"):
        return metrics

    weights = score_config.get("weights", {})
    score = 0.0
    for key, weight in weights.items():
        value = metrics.get(key)
        if value is None:
            return {"score": float("inf"), **metrics, "error": f"missing {key}"}
        score += float(weight) * value

    min_samples = int(score_config.get("min_samples", 0))
    if min_samples > 0 and metrics["samples"] < min_samples:
        shortfall = (min_samples - metrics["samples"]) / min_samples
        score += float(score_config.get("short_sample_penalty", 0.0)) * shortfall
        metrics["warning"] = f"used {metrics['samples']} samples below target {min_samples}"

    limit_warnings = []
    for key, limit in score_config.get("limits", {}).items():
        value = metrics.get(key)
        if value is None:
            return {"score": float("inf"), **metrics, "error": f"missing limit metric {key}"}
        maximum = float(limit.get("max", float("inf")))
        if value > maximum:
            excess = value - maximum
            score += float(limit.get("penalty", 0.0)) * excess
            limit_warnings.append(f"{key} {value:.4g} > {maximum:.4g}")
    if limit_warnings:
        previous_warning = metrics.get("warning")
        metrics["warning"] = (
            f"{previous_warning}; " if previous_warning else ""
        ) + "; ".join(limit_warnings)

    metrics["score"] = score
    return metrics


def newest_csv(directory):
    files = sorted(Path(directory).glob("tuning_*.csv"), key=lambda path: path.stat().st_mtime)
    return files[-1] if files else None


def append_status(candidate_dir, message):
    timestamp = time.strftime("%H:%M:%S")
    candidate_dir.mkdir(parents=True, exist_ok=True)
    with (candidate_dir / "status.txt").open("a") as handle:
        handle.write(f"[{timestamp}] {message}\n")


def run_bag_trial(config, candidate_dir, bag, trial_name, score_config=None):
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
            f"-p tuning_output_directory:={shlex.quote(str(csv_dir))} "
            "-p tuning_output_samples:=true"
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
        if bag.get("stop_after_bag_s") is not None:
            bag_timeout = float(bag["stop_after_bag_s"]) / playback_rate
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
            restore_files(original_global_config)

    csv_path = newest_csv(csv_dir)
    if csv_path is None:
        append_status(candidate_dir, f"{trial_name}: evaluator did not produce csv")
        remove_path_if_empty(csv_dir)
        return {"score": float("inf"), "error": "evaluator did not produce csv"}
    result = score_csv(csv_path, score_config or bag.get("score", config["tuning"]["score"]))
    if bag.get("track_name"):
        result["track_name"] = str(bag["track_name"])
    try:
        csv_path.unlink()
        remove_path_if_empty(csv_path.parent)
    except OSError as exc:
        append_status(candidate_dir, f"{trial_name}: could not remove evaluator csv: {exc}")
    append_status(candidate_dir, f"{trial_name}: score {result['score']}")
    return result


def run_candidate(
    config, parameters, values, run_dir, candidate_index, original_text, bags=None,
    score_config=None
):
    apply_values(parameters, values)
    candidate_dir = run_dir / "tries" / f"try_{candidate_index:04d}"
    append_status(candidate_dir, "starting try")
    append_status(
        candidate_dir,
        "parameters " + json.dumps(
            {spec["name"]: value for spec, value in zip(parameters, values)},
            sort_keys=True,
        ),
    )
    bag_results = []
    weighted_score = 0.0
    total_weight = 0.0
    try:
        selected_bags = bags if bags is not None else config["tuning"]["bags"]
        for bag_index, bag in enumerate(selected_bags):
            bag_score_config = resolve_score_config(
                score_config, bag, config["tuning"]["score"]
            )
            result = run_bag_trial(
                config, candidate_dir, bag, f"bag_{bag_index:02d}", bag_score_config
            )
            bag_results.append(result)
            weight = float(bag.get("weight", 1.0))
            weighted_score += weight * result["score"]
            total_weight += weight
        score = weighted_score / max(total_weight, 1e-9)
        status = "ok" if math.isfinite(score) else "failed"
        result = {
            "try": candidate_index,
            "status": status,
            "score": score,
            "values": {spec["name"]: value for spec, value in zip(parameters, values)},
            "bags": bag_results,
        }
        append_status(candidate_dir, f"finished try with status {status} and score {score}")
        return result
    finally:
        restore_files(original_text)


def baseline_values(parameters, config=None):
    initial_values = {}
    if config is not None:
        initial_values.update(config["tuning"].get("initial_values", {}))
        initial_values.update(config["tuning"].get("optimizer", {}).get("initial_values", {}))

    values = []
    for spec in parameters:
        if spec["name"] in initial_values:
            value = float(initial_values[spec["name"]])
            value = clamp(value, float(spec["min"]), float(spec["max"]))
        else:
            value = float(get_nested(load_yaml(WORKSPACE / spec["file"]), spec["path"]))
        values.append(value)
    return values


def random_values(parameters, rng):
    return [rng.uniform(float(spec["min"]), float(spec["max"])) for spec in parameters]


def clamp(value, low, high):
    return min(max(value, low), high)


def local_values(parameters, center, radius, rng):
    values = []
    for spec, value in zip(parameters, center):
        low = float(spec["min"])
        high = float(spec["max"])
        span = high - low
        step = rng.uniform(-radius, radius) * span
        values.append(clamp(value + step, low, high))
    return values


def coordinate_values(center, parameter_index, step):
    values = list(center)
    values[parameter_index] = values[parameter_index] + step
    return values


def parameter_indices(parameters, names):
    by_name = {spec["name"]: index for index, spec in enumerate(parameters)}
    missing = [name for name in names if name not in by_name]
    if missing:
        raise ValueError(f"Unknown tuning parameter(s): {', '.join(missing)}")
    return [by_name[name] for name in names]


def stage_bags(config, stage):
    bags = config["tuning"]["bags"]
    if "bag_indices" in stage:
        return [bags[int(index)] for index in stage["bag_indices"]]
    if "bag_names" in stage:
        wanted = set(stage["bag_names"])
        selected = [bag for bag in bags if bag.get("name") in wanted]
        if len(selected) != len(wanted):
            found = {bag.get("name") for bag in selected}
            missing = sorted(wanted - found)
            raise ValueError(f"Unknown bag name(s): {', '.join(missing)}")
        return selected
    return bags


def override_bag_stop_after(bags, stop_after_bag_s):
    overridden = []
    for bag in bags:
        bag_copy = dict(bag)
        if stop_after_bag_s is None:
            bag_copy.pop("stop_after_bag_s", None)
        else:
            requested_stop = float(stop_after_bag_s)
            if bag_copy.get("stop_after_bag_s") is not None:
                requested_stop = min(requested_stop, float(bag_copy["stop_after_bag_s"]))
            bag_copy["stop_after_bag_s"] = requested_stop
        overridden.append(bag_copy)
    return overridden


def resolve_score_config(score_config, bag, default_score_config):
    if score_config is None:
        return bag.get("score", default_score_config)
    if isinstance(score_config, dict) and "bag_scores" in score_config:
        bag_scores = score_config.get("bag_scores", {})
        bag_name = bag.get("name")
        if bag_name in bag_scores:
            return bag_scores[bag_name]
        return score_config.get("default", bag.get("score", default_score_config))
    return score_config


def randomize_indices(parameters, values, active_indices, rng):
    randomized = list(values)
    for index in active_indices:
        spec = parameters[index]
        randomized[index] = rng.uniform(float(spec["min"]), float(spec["max"]))
    return randomized


def mutate_individual(parameters, values, active_indices, rng, mutation_rate, mutation_scale):
    mutated = list(values)
    for index in active_indices:
        if rng.random() > mutation_rate:
            continue
        spec = parameters[index]
        low = float(spec["min"])
        high = float(spec["max"])
        span = high - low
        mutated[index] = clamp(mutated[index] + rng.gauss(0.0, mutation_scale * span), low, high)
    return mutated


def crossover_individuals(parameters, parent_a, parent_b, active_indices, rng, blend_alpha):
    child = list(parent_a)
    for index in active_indices:
        spec = parameters[index]
        low = float(spec["min"])
        high = float(spec["max"])
        alpha = rng.uniform(-blend_alpha, 1.0 + blend_alpha)
        child[index] = clamp(alpha * parent_a[index] + (1.0 - alpha) * parent_b[index], low, high)
    return child


def tournament_select(population, rng, tournament_size):
    contenders = rng.sample(population, min(tournament_size, len(population)))
    return min(contenders, key=lambda item: item["score"])


def flatten_result(result):
    row = {
        "try": result["try"],
        "kind": result.get("kind", ""),
        "status": result.get("status", ""),
        "score": result["score"],
    }
    for name, value in result["values"].items():
        row[f"param.{name}"] = value
    if result["bags"]:
        row["stage"] = result.get("stage", "")
        row["parameter"] = result.get("parameter", "")
        row["step_size"] = result.get("step_size", "")
        row["search_radius"] = result.get("search_radius", "")
        first_bag = result["bags"][0]
        for key in (
            "position_rmse",
            "heading_rmse",
            "velocity_rmse",
            "velocity_x_rmse",
            "velocity_x_mean_error",
            "velocity_x_slow_rmse",
            "velocity_x_slow_mean",
            "velocity_y_rmse",
            "yaw_rate_rmse",
            "yaw_rate_under_rmse",
            "front_wheel_rpm_rmse",
            "motor_rpm_rmse",
            "samples",
            "error",
            "warning",
        ):
            row[key] = first_bag.get(key, "")
        for key in sorted(first_bag):
            if key.endswith("_rmse") and key not in row:
                row[key] = first_bag.get(key, "")
    return row


def write_result(run_dir, result):
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
    dump_yaml(run_dir / "best.yaml", best_result)


def run_random(config, parameters, run_dir, original_text):
    optimizer = config["tuning"].get("optimizer", {})
    rng = random.Random(int(optimizer.get("seed", 7)))
    trials = int(optimizer.get("trials", 24))

    best = None
    candidate_index = 0

    values = baseline_values(parameters, config)
    result = run_candidate(config, parameters, values, run_dir, candidate_index, original_text)
    result["kind"] = "baseline"
    write_result(run_dir, result)
    best = result
    candidate_index += 1

    for _ in range(trials):
        values = random_values(parameters, rng)
        result = run_candidate(config, parameters, values, run_dir, candidate_index, original_text)
        result["kind"] = "random"
        write_result(run_dir, result)
        if result["score"] < best["score"]:
            best = result
            copy_best_csvs(run_dir, best)
        candidate_index += 1
    return best


def run_local_refinement(config, parameters, run_dir, original_text):
    optimizer = config["tuning"].get("optimizer", {})
    rng = random.Random(int(optimizer.get("seed", 7)))
    trials = int(optimizer.get("trials", 24))
    radius = float(optimizer.get("initial_radius", 0.25))
    min_radius = float(optimizer.get("min_radius", 0.02))
    shrink = float(optimizer.get("shrink", 0.6))
    patience = max(1, int(optimizer.get("patience", 4)))

    candidate_index = 0
    best_values = baseline_values(parameters, config)
    best = run_candidate(config, parameters, best_values, run_dir, candidate_index, original_text)
    best["kind"] = "baseline"
    write_result(run_dir, best)
    copy_best_csvs(run_dir, best)
    candidate_index += 1

    attempts_without_improvement = 0
    for _ in range(trials):
        values = local_values(parameters, best_values, radius, rng)
        result = run_candidate(config, parameters, values, run_dir, candidate_index, original_text)
        result["kind"] = "local_refinement"
        result["search_radius"] = radius
        write_result(run_dir, result)

        if result["score"] < best["score"]:
            best = result
            best_values = values
            attempts_without_improvement = 0
            copy_best_csvs(run_dir, best)
        else:
            attempts_without_improvement += 1
            if attempts_without_improvement >= patience:
                radius = max(min_radius, radius * shrink)
                attempts_without_improvement = 0
        candidate_index += 1
    return best


def run_coordinate_refinement(config, parameters, run_dir, original_text):
    optimizer = config["tuning"].get("optimizer", {})
    trials = int(optimizer.get("trials", 40))
    initial_step = float(optimizer.get("initial_step", 0.20))
    min_step = float(optimizer.get("min_step", 0.01))
    shrink = float(optimizer.get("shrink", 0.5))

    candidate_index = 0
    best_values = baseline_values(parameters, config)
    best = run_candidate(config, parameters, best_values, run_dir, candidate_index, original_text)
    best["kind"] = "baseline"
    write_result(run_dir, best)
    copy_best_csvs(run_dir, best)
    candidate_index += 1

    step_sizes = []
    for spec in parameters:
        step_sizes.append((float(spec["max"]) - float(spec["min"])) * initial_step)

    while candidate_index <= trials and max(step_sizes) > 0.0:
        improved_this_pass = False

        for parameter_index, spec in enumerate(parameters):
            if candidate_index > trials:
                break
            if step_sizes[parameter_index] <= 0.0:
                continue

            low = float(spec["min"])
            high = float(spec["max"])
            span = high - low
            directions = (1.0, -1.0)
            improved_parameter = False

            for direction in directions:
                if candidate_index > trials:
                    break

                candidate_value = clamp(
                    best_values[parameter_index] + direction * step_sizes[parameter_index],
                    low,
                    high,
                )
                if candidate_value == best_values[parameter_index]:
                    continue

                values = list(best_values)
                values[parameter_index] = candidate_value
                result = run_candidate(
                    config, parameters, values, run_dir, candidate_index, original_text
                )
                result["kind"] = "coordinate_refinement"
                result["parameter"] = spec["name"]
                result["step_size"] = step_sizes[parameter_index]
                write_result(run_dir, result)

                if result["score"] < best["score"]:
                    best = result
                    best_values = values
                    improved_this_pass = True
                    improved_parameter = True
                    copy_best_csvs(run_dir, best)
                    break

                candidate_index += 1

            if improved_parameter:
                candidate_index += 1
                continue

            step_sizes[parameter_index] *= shrink
            if step_sizes[parameter_index] < span * min_step:
                step_sizes[parameter_index] = 0.0

        if not improved_this_pass and all(step_size == 0.0 for step_size in step_sizes):
            break

    return best


def run_staged_hybrid(config, parameters, run_dir, original_text):
    optimizer = config["tuning"].get("optimizer", {})
    stages = optimizer.get("stages", [])
    if not stages:
        raise ValueError("staged_hybrid optimizer requires optimizer.stages")

    rng = random.Random(int(optimizer.get("seed", 7)))
    candidate_index = 0
    best_values = baseline_values(parameters, config)
    final_best = None

    for stage in stages:
        stage_name = stage["name"]
        active_indices = parameter_indices(parameters, stage["parameter_names"])
        selected_bags = stage_bags(config, stage)
        score_config = stage.get("score", config["tuning"]["score"])
        trials = int(stage.get("trials", optimizer.get("trials", 40)))
        initial_step = float(stage.get("initial_step", optimizer.get("initial_step", 0.18)))
        min_step = float(stage.get("min_step", optimizer.get("min_step", 0.01)))
        shrink = float(stage.get("shrink", optimizer.get("shrink", 0.5)))
        exploration_interval = int(
            stage.get("exploration_interval", optimizer.get("exploration_interval", 10))
        )

        stage_start_index = candidate_index
        stage_best = run_candidate(
            config, parameters, best_values, run_dir, candidate_index, original_text,
            selected_bags, score_config
        )
        stage_best["kind"] = "stage_baseline"
        stage_best["stage"] = stage_name
        write_result(run_dir, stage_best)
        copy_best_csvs(run_dir, stage_best)
        candidate_index += 1

        step_sizes = [0.0 for _ in parameters]
        for index in active_indices:
            spec = parameters[index]
            step_sizes[index] = (float(spec["max"]) - float(spec["min"])) * initial_step

        while candidate_index - stage_start_index <= trials:
            improved_this_pass = False

            for parameter_index in active_indices:
                if candidate_index - stage_start_index > trials:
                    break
                if step_sizes[parameter_index] <= 0.0:
                    continue

                spec = parameters[parameter_index]
                low = float(spec["min"])
                high = float(spec["max"])
                span = high - low
                improved_parameter = False

                for direction in (1.0, -1.0):
                    if candidate_index - stage_start_index > trials:
                        break

                    candidate_value = clamp(
                        best_values[parameter_index] + direction * step_sizes[parameter_index],
                        low,
                        high,
                    )
                    if candidate_value == best_values[parameter_index]:
                        continue

                    values = list(best_values)
                    values[parameter_index] = candidate_value
                    result = run_candidate(
                        config, parameters, values, run_dir, candidate_index, original_text,
                        selected_bags, score_config
                    )
                    result["kind"] = "staged_coordinate"
                    result["stage"] = stage_name
                    result["parameter"] = spec["name"]
                    result["step_size"] = step_sizes[parameter_index]
                    write_result(run_dir, result)

                    if result["score"] < stage_best["score"]:
                        stage_best = result
                        best_values = values
                        final_best = result
                        improved_this_pass = True
                        improved_parameter = True
                        copy_best_csvs(run_dir, stage_best)
                        break

                    candidate_index += 1

                if improved_parameter:
                    candidate_index += 1
                    continue

                step_sizes[parameter_index] *= shrink
                if step_sizes[parameter_index] < span * min_step:
                    step_sizes[parameter_index] = 0.0

            stage_tries = candidate_index - stage_start_index
            if exploration_interval > 0 and stage_tries > 0 and stage_tries % exploration_interval == 0:
                values = randomize_indices(parameters, best_values, active_indices, rng)
                result = run_candidate(
                    config, parameters, values, run_dir, candidate_index, original_text,
                    selected_bags, score_config
                )
                result["kind"] = "staged_exploration"
                result["stage"] = stage_name
                write_result(run_dir, result)
                if result["score"] < stage_best["score"]:
                    stage_best = result
                    best_values = values
                    final_best = result
                    copy_best_csvs(run_dir, stage_best)
                    for index in active_indices:
                        spec = parameters[index]
                        step_sizes[index] = (
                            float(spec["max"]) - float(spec["min"])
                        ) * initial_step
                candidate_index += 1

            if not improved_this_pass and all(step_sizes[index] == 0.0 for index in active_indices):
                break

        final_best = stage_best

    if optimizer.get("final_validation", True):
        final_result = run_candidate(
            config, parameters, best_values, run_dir, candidate_index, original_text
        )
        final_result["kind"] = "final_validation"
        final_result["stage"] = "all_bags"
        write_result(run_dir, final_result)
        copy_best_csvs(run_dir, final_result)
        final_best = final_result

    return final_best


def run_staged_genetic(config, parameters, run_dir, original_text):
    optimizer = config["tuning"].get("optimizer", {})
    stages = optimizer.get("stages", [])
    if not stages:
        raise ValueError("staged_genetic optimizer requires optimizer.stages")

    rng = random.Random(int(optimizer.get("seed", 7)))
    candidate_index = 0
    best_values = baseline_values(parameters, config)
    final_best = None

    for stage in stages:
        stage_name = stage["name"]
        active_indices = parameter_indices(parameters, stage["parameter_names"])
        selected_bags = stage_bags(config, stage)
        score_config = stage.get("score", config["tuning"]["score"])
        generations = int(stage.get("generations", optimizer.get("generations", 8)))
        population_size = int(stage.get("population_size", optimizer.get("population_size", 24)))
        elite_count = max(1, int(stage.get("elite_count", optimizer.get("elite_count", 4))))
        immigrant_count = int(stage.get("immigrant_count", optimizer.get("immigrant_count", 3)))
        mutation_rate = float(stage.get("mutation_rate", optimizer.get("mutation_rate", 0.35)))
        mutation_scale = float(stage.get("mutation_scale", optimizer.get("mutation_scale", 0.16)))
        blend_alpha = float(stage.get("blend_alpha", optimizer.get("blend_alpha", 0.25)))
        tournament_size = int(stage.get("tournament_size", optimizer.get("tournament_size", 3)))
        local_seed_count = int(stage.get("local_seed_count", optimizer.get("local_seed_count", 6)))
        local_seed_radius = float(stage.get("local_seed_radius", optimizer.get("local_seed_radius", 0.12)))
        long_eval_top_k = int(stage.get("long_eval_top_k", 0))
        long_eval_stop_after_bag_s = stage.get("long_eval_stop_after_bag_s")

        population_values = [list(best_values)]
        for _ in range(local_seed_count):
            population_values.append(
                mutate_individual(
                    parameters, best_values, active_indices, rng, 1.0, local_seed_radius
                )
            )
        while len(population_values) < population_size:
            population_values.append(randomize_indices(parameters, best_values, active_indices, rng))

        stage_best = None
        for generation in range(generations):
            evaluated = []
            for values in population_values[:population_size]:
                result = run_candidate(
                    config, parameters, values, run_dir, candidate_index, original_text,
                    selected_bags, score_config
                )
                result["kind"] = "staged_genetic"
                result["stage"] = stage_name
                result["generation"] = generation
                write_result(run_dir, result)
                evaluated.append({"score": result["score"], "values": values, "result": result})

                if stage_best is None or result["score"] < stage_best["score"]:
                    stage_best = result
                    best_values = values
                    final_best = result
                    copy_best_csvs(run_dir, stage_best)
                candidate_index += 1

            evaluated.sort(key=lambda item: item["score"])
            if long_eval_top_k > 0 and long_eval_stop_after_bag_s is not None:
                long_eval_bags = override_bag_stop_after(
                    selected_bags, float(long_eval_stop_after_bag_s)
                )
                for item in evaluated[:long_eval_top_k]:
                    result = run_candidate(
                        config, parameters, item["values"], run_dir, candidate_index,
                        original_text, long_eval_bags, score_config
                    )
                    result["kind"] = "staged_genetic_long_eval"
                    result["stage"] = stage_name
                    result["generation"] = generation
                    write_result(run_dir, result)

                    item["score"] = result["score"]
                    item["result"] = result
                    if stage_best is None or result["score"] < stage_best["score"]:
                        stage_best = result
                        best_values = item["values"]
                        final_best = result
                        copy_best_csvs(run_dir, stage_best)
                    candidate_index += 1

            evaluated.sort(key=lambda item: item["score"])
            elites = evaluated[:elite_count]
            next_population = [list(item["values"]) for item in elites]

            for _ in range(immigrant_count):
                if len(next_population) >= population_size:
                    break
                next_population.append(
                    randomize_indices(parameters, best_values, active_indices, rng)
                )

            while len(next_population) < population_size:
                parent_a = tournament_select(evaluated, rng, tournament_size)["values"]
                parent_b = tournament_select(evaluated, rng, tournament_size)["values"]
                child = crossover_individuals(
                    parameters, parent_a, parent_b, active_indices, rng, blend_alpha
                )
                child = mutate_individual(
                    parameters, child, active_indices, rng, mutation_rate, mutation_scale
                )
                next_population.append(child)

            population_values = next_population

        final_best = stage_best

    if optimizer.get("final_validation", True):
        final_result = run_candidate(
            config, parameters, best_values, run_dir, candidate_index, original_text
        )
        final_result["kind"] = "final_validation"
        final_result["stage"] = "all_bags"
        write_result(run_dir, final_result)
        copy_best_csvs(run_dir, final_result)
        final_best = final_result

    return final_best


def run_differential_evolution(config, parameters, run_dir, original_text):
    from scipy.optimize import differential_evolution

    optimizer = config["tuning"].get("optimizer", {})
    bounds = [(float(spec["min"]), float(spec["max"])) for spec in parameters]
    best = {"score": float("inf")}
    counter = {"value": 0}

    values = baseline_values(parameters, config)
    baseline = run_candidate(config, parameters, values, run_dir, counter["value"], original_text)
    baseline["kind"] = "baseline"
    write_result(run_dir, baseline)
    best = baseline
    counter["value"] += 1

    def objective(vector):
        nonlocal best
        result = run_candidate(
            config, parameters, list(vector), run_dir, counter["value"], original_text
        )
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
    parser.add_argument(
        "--optimizer",
        choices=["random", "de", "local", "coordinate", "staged_hybrid", "staged_genetic"],
        default=""
    )
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
        DEFAULT_TUNING_ROOT / time.strftime("tuning_%Y%m%d_%H%M%S")
    )
    run_dir.mkdir(parents=True, exist_ok=True)

    touched_files = {WORKSPACE / spec["file"] for spec in parameters}
    original_text = {path: path.read_text() for path in touched_files}

    best = None
    try:
        method = config["tuning"].get("optimizer", {}).get("method", "random")
        if method == "de":
            best = run_differential_evolution(config, parameters, run_dir, original_text)
        elif method == "staged_genetic":
            best = run_staged_genetic(config, parameters, run_dir, original_text)
        elif method == "staged_hybrid":
            best = run_staged_hybrid(config, parameters, run_dir, original_text)
        elif method == "coordinate":
            best = run_coordinate_refinement(config, parameters, run_dir, original_text)
        elif method == "local":
            best = run_local_refinement(config, parameters, run_dir, original_text)
        else:
            best = run_random(config, parameters, run_dir, original_text)
        copy_best_csvs(run_dir, best)
    finally:
        restore_files(original_text)

    if best and args.apply_best:
        apply_values(parameters, [best["values"][spec["name"]] for spec in parameters])

    print(json.dumps({"run_dir": str(run_dir), "best": best}, indent=2))


if __name__ == "__main__":
    main()
