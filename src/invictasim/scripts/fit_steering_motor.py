#!/usr/bin/env python3

import argparse
import bisect
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


CONTROL_TOPIC = "/control/command"
STEERING_TOPIC = "/vehicle/steering_motor_state"


def message_type_map(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def read_samples(bag_path, control_topic, steering_topic):
    types = message_type_map(bag_path)
    if control_topic not in types:
        raise RuntimeError(f"{control_topic} not found in {bag_path}")
    if steering_topic not in types:
        raise RuntimeError(f"{steering_topic} not found in {bag_path}")

    control_type = get_message(types[control_topic])
    steering_type = get_message(types[steering_topic])

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    commands = []
    steering = []
    first_time = None
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in (control_topic, steering_topic):
            continue
        if first_time is None:
            first_time = timestamp
        t = (timestamp - first_time) * 1e-9
        if topic == control_topic:
            msg = deserialize_message(data, control_type)
            commands.append((t, float(msg.steering)))
        else:
            msg = deserialize_message(data, steering_type)
            steering.append((t, float(msg.steering_angle)))
    return commands, steering


def command_at(commands, t):
    index = bisect.bisect_right(commands, (t, float("inf"))) - 1
    if index < 0:
        return commands[0][1]
    return commands[index][1]


def filter_window(commands, steering, start_s, duration_s, min_abs_steering):
    if start_s is not None:
        steering = [(t, value) for t, value in steering if t >= start_s]
    if duration_s is not None and steering:
        t0 = steering[0][0]
        steering = [(t, value) for t, value in steering if t <= t0 + duration_s]
    if min_abs_steering > 0.0:
        steering = [
            (t, value)
            for t, value in steering
            if abs(value) >= min_abs_steering or abs(command_at(commands, t)) >= min_abs_steering
        ]
    return steering


def simulate_first_order(commands, sample_times, tau, initial_value):
    values = []
    current = initial_value
    last_t = sample_times[0]
    for t in sample_times:
        dt = max(0.0, t - last_t)
        target = command_at(commands, t)
        if tau <= 1e-9:
            current = target
        else:
            alpha = 1.0 - math.exp(-dt / tau)
            current += alpha * (target - current)
        values.append(current)
        last_t = t
    return values


def rmse(errors):
    if not errors:
        return float("inf")
    return math.sqrt(sum(error * error for error in errors) / len(errors))


def score_tau(commands, steering, tau):
    sample_times = [t for t, _ in steering]
    real_values = [value for _, value in steering]
    simulated = simulate_first_order(commands, sample_times, tau, real_values[0])
    return rmse([sim - real for sim, real in zip(simulated, real_values)])


def linear_fit(xs, ys):
    n = len(xs)
    if n == 0:
        return None
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    var_x = sum((x - mean_x) ** 2 for x in xs)
    if var_x <= 1e-12:
        return None
    cov_xy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    scale = cov_xy / var_x
    offset = mean_y - scale * mean_x
    errors = [scale * x + offset - y for x, y in zip(xs, ys)]
    return {
        "scale": scale,
        "offset": offset,
        "rmse": rmse(errors),
    }


def fit_time_constant(commands, steering, tau_min, tau_max):
    best = {"time_constant": None, "rmse": float("inf")}
    low = tau_min
    high = tau_max
    for _ in range(5):
        steps = 80
        candidates = [low + (high - low) * i / steps for i in range(steps + 1)]
        for tau in candidates:
            score = score_tau(commands, steering, tau)
            if score < best["rmse"]:
                best = {"time_constant": tau, "rmse": score}
        span = (high - low) / 8.0
        low = max(tau_min, best["time_constant"] - span)
        high = min(tau_max, best["time_constant"] + span)
    return best


def main():
    parser = argparse.ArgumentParser(
        description="Fit InvictaSim steering_motor.time_constant from steering command and sensor data."
    )
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--control-topic", default=CONTROL_TOPIC)
    parser.add_argument("--steering-topic", default=STEERING_TOPIC)
    parser.add_argument("--tau-min", type=float, default=0.005)
    parser.add_argument("--tau-max", type=float, default=0.30)
    parser.add_argument("--start-s", type=float, default=None)
    parser.add_argument("--duration-s", type=float, default=None)
    parser.add_argument("--min-abs-steering", type=float, default=0.01)
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    all_commands = []
    all_steering = []
    per_bag = []
    time_offset = 0.0
    for bag in args.bags:
        commands, steering = read_samples(Path(bag), args.control_topic, args.steering_topic)
        steering = filter_window(commands, steering, args.start_s, args.duration_s, args.min_abs_steering)
        if not commands or not steering:
            raise RuntimeError(f"Not enough steering data in {bag}")
        shifted_commands = [(t + time_offset, value) for t, value in commands]
        shifted_steering = [(t + time_offset, value) for t, value in steering]
        all_commands.extend(shifted_commands)
        all_steering.extend(shifted_steering)
        time_offset = max(t for t, _ in shifted_commands + shifted_steering) + 1.0
        per_bag.append(
            {
                "bag": bag,
                "command_samples": len(commands),
                "steering_samples_used": len(steering),
                "duration_used_s": steering[-1][0] - steering[0][0],
            }
        )

    all_commands.sort()
    all_steering.sort()
    best = fit_time_constant(all_commands, all_steering, args.tau_min, args.tau_max)

    command_values = [command_at(all_commands, t) for t, _ in all_steering]
    steering_values = [value for _, value in all_steering]
    direct_linear = linear_fit(command_values, steering_values)

    result = {
        "best_time_constant": best["time_constant"],
        "time_constant_rmse_rad": best["rmse"],
        "direct_command_to_sensor_linear_fit": direct_linear,
        "control_topic": args.control_topic,
        "steering_topic": args.steering_topic,
        "samples": len(all_steering),
        "bags": per_bag,
    }
    text = json.dumps(result, indent=2)
    if args.output:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output).write_text(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
