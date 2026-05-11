#!/usr/bin/env python3
"""
Track-aware staged Bayesian optimization for fitting InvictaSim to a rosbag.

This script keeps the same speedup-based simulator/bag pipeline as
bayes_fit_invictasim.py, but splits the search into phases:

1. Drivetrain on mostly-straight sections.
2. Steering/yaw on curved sections.
3. Differential on powered curved sections.
4. Full local refinement without tire parameters, still using MF 6.2.
5. Switch to simple Pacejka combined slip and tune tire parameters, only once
   the non-tire fit is good enough.

It writes to its own results directory and does not delete or reuse histories from
the other optimizers.
"""

from __future__ import annotations

import argparse
import atexit
import json
import math
import os
import shlex
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import yaml
from custom_interfaces.msg import ControlCommand, Pose, Velocities
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import bayes_fit_invictasim as base


# =============================================================================
# User-tunable constants
# =============================================================================

ROOT = Path(__file__).resolve().parent
BAG_PATH = ROOT / "bags" / "DV Trackdrive 5 (Best one yet).mcap"
RESULTS_DIR = ROOT / "bo_results" / "invictasim_fit_staged_europarque"
WARM_START_PARAMETERS = ROOT / "bo_results" / "invictasim_fit" / "best_parameters.yaml"
STAGED_WARM_START_PARAMETERS = ROOT / "bo_results" / "invictasim_fit_staged_warm_start.yaml"
MF6_2_NON_TIRE_WARM_START_PARAMETERS = (
    ROOT / "bo_results" / "invictasim_fit_no_tire" / "best_parameters.yaml"
)
USE_MF6_2_NON_TIRE_WARM_START = False

SIMULATION_SPEEDUP = base.SIMULATION_SPEEDUP
INITIAL_TIRE_MODEL = "pacejka_MF6_2"
FINAL_TIRE_MODEL = "pacejka_combined_slip"

INITIAL_EVAL_DURATION_S = 45.0
MAX_TRIALS = 40000
RANDOM_SEED = 19
RESUME_THIS_SCRIPT_RESULTS = True
RESTORE_SIM_SPEED_ON_EXIT = True

SAMPLE_RATE_HZ = 50.0
MIN_OVERLAP_S = 5.0
MIN_MASK_SAMPLES = 30
POST_PLAY_DRAIN_S = base.POST_PLAY_DRAIN_S

# Track masks. These are intentionally based on the real bag, so the optimizer
# understands which sections are straights and which are curves.
STRAIGHT_YAWRATE_MAX_RAD_S = 0.18
STRAIGHT_STEER_MAX_RAD = 0.08
CURVE_YAWRATE_MIN_RAD_S = 0.22
CURVE_STEER_MIN_RAD = 0.09
ACCEL_THROTTLE_MIN = 0.08
COAST_THROTTLE_MAX = 0.035
DIFF_THROTTLE_MIN = 0.04
MIN_VX_FOR_DYNAMIC_MASKS_MPS = 1.0

# Bad trials can be cut short after enough effective time has elapsed. Keep the
# thresholds generous because early phases are not scored mainly on pose.
ENABLE_EARLY_STOP = True
EARLY_STOP_AFTER_EFFECTIVE_S = 8.0
EARLY_STOP_CHECK_PERIOD_WALL_S = 0.35
EARLY_STOP_POSITION_MAX_M = 35.0
EARLY_STOP_VX_MAX_MPS = 8.0
EARLY_STOP_YAWRATE_MAX_RAD_S = 3.0

# The tire phase is deliberately delayed. Set this higher if you want tire tuning
# to start sooner, or set FORCE_TIRE_PHASE=True to always run it.
FORCE_TIRE_PHASE = False
TIRE_PHASE_REQUIRES_POSITION_MAX_BELOW_M = 4.0

# The staged fit is warm-started, so each phase should search locally first.
# Fractions are relative to each parameter's full allowed range.
DEFAULT_PHASE_LOCAL_RADIUS = 0.10
DEFAULT_PHASE_INITIAL_RANDOM_TRIALS = 8
DEFAULT_PHASE_MIN_TRIALS = 12
DEFAULT_PHASE_PATIENCE_TRIALS = 18
DEFAULT_PHASE_MIN_RELATIVE_IMPROVEMENT = 0.006

# Even in specialized phases, keep a small global-drift guard in the objective so
# a candidate cannot "win" straights while throwing the car far off the track.
GLOBAL_GUARD_SCORE_WEIGHTS = {
    "drivetrain": {"position_rmse": 0.10, "position_max": 0.045, "yawrate_rmse": 0.20},
    "steering": {"position_rmse": 0.08, "position_max": 0.06, "vx_rmse": 0.15},
    "differential": {"position_rmse": 0.10, "position_max": 0.07, "yaw_rmse": 0.25},
    "tire": {"vx_max": 0.05, "yawrate_max": 0.08},
    "full": {},
}

DRIVETRAIN_PARAMS = [
    "motor.max_peak_torque",
    "motor.max_continous_torque",
    "motor.throttle_torque_curve.0.1",
    "motor.throttle_torque_curve.0.2",
    "motor.throttle_torque_curve.0.3",
    "motor.throttle_torque_curve.0.4",
    "motor.throttle_torque_curve.0.5",
    "motor.throttle_torque_curve.0.6",
    "motor.throttle_torque_curve.0.7",
    "motor.throttle_torque_curve.0.8",
    "motor.throttle_torque_curve.0.9",
    "transmission.efficiency",
    "transmission.viscous_drag_coeff",
    "transmission.coulomb_drag",
    "car.front_bearing_drag",
]

STEERING_PARAMS = [
    "steering_motor.time_constant",
    "steering.ackerman_factor",
    "car.Izz",
]

DIFFERENTIAL_PARAMS = [
    "transmission.kv",
    "transmission.t_max",
]

TIRE_PARAMS = [
    "tire.B_lateral",
    "tire.C_lateral",
    "tire.D_lateral",
    "tire.E_lateral",
    "tire.B_longitudinal",
    "tire.C_longitudinal",
    "tire.D_longitudinal",
    "tire.E_longitudinal",
    "tire.relaxation_length",
    "tire.effective_tire_r",
]

NON_TIRE_PARAMS = DRIVETRAIN_PARAMS + STEERING_PARAMS + DIFFERENTIAL_PARAMS

PHASE_SCHEDULE = [
    {
        "name": "drivetrain_straights",
        "trials": 500,
        "params": DRIVETRAIN_PARAMS,
        "score_mode": "drivetrain",
        "local_radius": 0.2,
        "initial_random_trials": 8,
        "min_trials": 14,
        "patience": 50,
    },
    {
        "name": "steering_curves",
        "trials": 200,
        "params": STEERING_PARAMS,
        "score_mode": "steering",
        "local_radius": 0.12,
        "initial_random_trials": 7,
        "min_trials": 12,
        "patience": 30,
    },
    {
        "name": "differential_powered_curves",
        "trials": 200,
        "params": DIFFERENTIAL_PARAMS,
        "score_mode": "differential",
        "local_radius": 0.16,
        "initial_random_trials": 6,
        "min_trials": 10,
        "patience": 20,
    },
    {
        "name": "full_local_no_tire",
        "trials": 10000,
        "params": NON_TIRE_PARAMS,
        "score_mode": "full",
        "local_radius": 0.10,
        "initial_random_trials": 10,
        "min_trials": 18,
        "patience": 500,
    },
    {
        "name": "final_simple_pacejka_tire",
        "trials": 1000,
        "params": TIRE_PARAMS,
        "score_mode": "tire",
        "requires_position_max_below_m": TIRE_PHASE_REQUIRES_POSITION_MAX_BELOW_M,
        "local_radius": 0.12,
        "initial_random_trials": 10,
        "min_trials": 20,
        "patience": 100,
    },
]


# =============================================================================
# Data structures
# =============================================================================


@dataclass
class PhaseTrialResult:
    trial: int
    phase: str
    phase_trial: int
    score: float
    duration_s: float
    metrics: dict[str, float]
    params: dict[str, float]


# =============================================================================
# Parameter helpers
# =============================================================================


def all_enabled_params() -> list[dict[str, Any]]:
    return [spec for spec in base.PARAM_SPACE if spec.get("enabled", True)]


def params_by_name(names: list[str]) -> list[dict[str, Any]]:
    specs = {spec["name"]: spec for spec in all_enabled_params()}
    missing = [name for name in names if name not in specs]
    if missing:
        raise RuntimeError(f"phase references unknown/disabled parameters: {missing}")
    return [specs[name] for name in names]


def local_param_specs(
    specs: list[dict[str, Any]],
    center: dict[str, float],
    radius: float,
) -> list[dict[str, Any]]:
    local_specs = []
    for spec in specs:
        lo, hi = spec["bounds"]
        span = hi - lo
        value = center[spec["name"]]
        local = spec.copy()
        local["bounds"] = [
            max(lo, value - radius * span),
            min(hi, value + radius * span),
        ]
        local_specs.append(local)
    return local_specs


def subset(params: dict[str, float], specs: list[dict[str, Any]]) -> dict[str, float]:
    return {spec["name"]: float(params[spec["name"]]) for spec in specs}


def params_within_specs(params: dict[str, float], specs: list[dict[str, Any]], tol: float = 1e-9) -> bool:
    for spec in specs:
        lo, hi = spec["bounds"]
        value = params[spec["name"]]
        if value < lo - tol or value > hi + tol:
            return False
    return True


def optimizer_observation_params(
    params: dict[str, float],
    optimizer_specs: list[dict[str, Any]],
) -> dict[str, float]:
    values = subset(params, optimizer_specs)
    return base.clamp_params_to_bounds(values, optimizer_specs)


def load_yaml_params(path: Path) -> dict[str, float]:
    if not path.exists():
        return {}
    data = base.load_yaml(path)
    return {str(k): float(v) for k, v in data.items()}


def apply_named_values(
    values: dict[str, float],
    source: dict[str, float],
    allowed_names: set[str] | None = None,
) -> None:
    for name, value in source.items():
        if name not in values:
            continue
        if allowed_names is not None and name not in allowed_names:
            continue
        values[name] = value


def load_initial_params(params: list[dict[str, Any]]) -> dict[str, float]:
    values = base.load_seed_params(params)
    if values is None:
        values = base.read_current_params(params)

    # Use the all-parameter fit as the safe full-car baseline. The no-tire/MF
    # result can be overlaid manually, but it is not always a better complete
    # starting point for this bag segment.
    apply_named_values(values, load_yaml_params(WARM_START_PARAMETERS))
    apply_named_values(values, load_yaml_params(STAGED_WARM_START_PARAMETERS))
    if USE_MF6_2_NON_TIRE_WARM_START:
        apply_named_values(
            values,
            load_yaml_params(MF6_2_NON_TIRE_WARM_START_PARAMETERS),
            set(NON_TIRE_PARAMS),
        )

    own_best = RESULTS_DIR / "best_parameters.yaml"
    if RESUME_THIS_SCRIPT_RESULTS and own_best.exists():
        apply_named_values(values, load_yaml_params(own_best))

    return base.clamp_params_to_bounds(values, params)


def merge_phase_params(
    current: dict[str, float],
    source: dict[str, float],
    phase_specs: list[dict[str, Any]],
    all_params: list[dict[str, Any]],
) -> dict[str, float]:
    merged = current.copy()
    for spec in phase_specs:
        name = spec["name"]
        if name in source:
            merged[name] = source[name]
    return base.clamp_params_to_bounds(merged, all_params)


def configure_base_globals(results_dir: Path, bag_path: Path) -> None:
    base.RESULTS_DIR = results_dir
    base.BAG_PATH = bag_path
    base.TIRE_MODEL = INITIAL_TIRE_MODEL
    base.RESTORE_SIM_SPEED_ON_EXIT = RESTORE_SIM_SPEED_ON_EXIT


def set_active_tire_model(tire_model: str) -> None:
    base.TIRE_MODEL = tire_model
    base.set_tire_model()


def tire_model_for_phase(phase: dict[str, Any]) -> str:
    if str(phase["score_mode"]) == "tire":
        return FINAL_TIRE_MODEL
    return INITIAL_TIRE_MODEL


# =============================================================================
# ROS data collection
# =============================================================================


class TrackAwareCollector(Node):
    def __init__(self) -> None:
        super().__init__("invictasim_staged_bayes_fit_collector")
        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        self.real_pose: list[tuple[float, float, float, float]] = []
        self.real_vel: list[tuple[float, float, float]] = []
        self.sim_pose: list[tuple[float, float, float, float]] = []
        self.sim_vel: list[tuple[float, float, float]] = []
        self.commands: list[tuple[float, float, float]] = []

        self.create_subscription(Pose, base.REAL_POSE_TOPIC, self._real_pose_cb, 300)
        self.create_subscription(Velocities, base.REAL_VELOCITY_TOPIC, self._real_vel_cb, 300)
        self.create_subscription(Pose, base.SIM_POSE_TOPIC, self._sim_pose_cb, 300)
        self.create_subscription(Velocities, base.SIM_VELOCITY_TOPIC, self._sim_vel_cb, 300)
        self.create_subscription(ControlCommand, "/control/command", self._command_cb, 300)

    def reset(self) -> None:
        with self.lock:
            self.t0 = time.monotonic()
            self.real_pose.clear()
            self.real_vel.clear()
            self.sim_pose.clear()
            self.sim_vel.clear()
            self.commands.clear()

    def snapshot(self) -> dict[str, np.ndarray]:
        with self.lock:
            return {
                "real_pose": np.asarray(self.real_pose, dtype=float),
                "real_vel": np.asarray(self.real_vel, dtype=float),
                "sim_pose": np.asarray(self.sim_pose, dtype=float),
                "sim_vel": np.asarray(self.sim_vel, dtype=float),
                "commands": np.asarray(self.commands, dtype=float),
            }

    def counts(self) -> dict[str, int]:
        with self.lock:
            return {
                "real_pose": len(self.real_pose),
                "real_vel": len(self.real_vel),
                "sim_pose": len(self.sim_pose),
                "sim_vel": len(self.sim_vel),
                "commands": len(self.commands),
            }

    def elapsed(self) -> float:
        with self.lock:
            return time.monotonic() - self.t0

    def _stamp(self) -> float:
        return time.monotonic() - self.t0

    def _real_pose_cb(self, msg: Pose) -> None:
        with self.lock:
            self.real_pose.append((self._stamp(), msg.x, msg.y, base.normalize_angle(msg.theta)))

    def _real_vel_cb(self, msg: Velocities) -> None:
        with self.lock:
            self.real_vel.append((self._stamp(), msg.velocity_x, msg.angular_velocity))

    def _sim_pose_cb(self, msg: Pose) -> None:
        with self.lock:
            self.sim_pose.append((self._stamp(), msg.x, msg.y, base.normalize_angle(msg.theta)))

    def _sim_vel_cb(self, msg: Velocities) -> None:
        with self.lock:
            self.sim_vel.append((self._stamp(), msg.velocity_x, msg.angular_velocity))

    def _command_cb(self, msg: ControlCommand) -> None:
        throttle = 0.25 * (msg.throttle_fl + msg.throttle_fr + msg.throttle_rl + msg.throttle_rr)
        with self.lock:
            self.commands.append((self._stamp(), throttle, msg.steering))


def start_collector() -> tuple[TrackAwareCollector, SingleThreadedExecutor, threading.Thread]:
    if not rclpy.ok():
        rclpy.init(args=None)
    collector = TrackAwareCollector()
    executor = SingleThreadedExecutor()
    executor.add_node(collector)
    thread = threading.Thread(target=executor.spin, daemon=False)
    thread.start()
    return collector, executor, thread


# =============================================================================
# Metrics
# =============================================================================


def interpolate_series(data: np.ndarray, t: np.ndarray, value_col: int, unwrap: bool = False) -> np.ndarray:
    values = data[:, value_col]
    if unwrap:
        values = np.unwrap(values)
    return np.interp(t, data[:, 0], values)


def masked_rmse(values: np.ndarray, mask: np.ndarray, fallback_mask: np.ndarray | None = None) -> float:
    if int(np.count_nonzero(mask)) < MIN_MASK_SAMPLES:
        if fallback_mask is not None and int(np.count_nonzero(fallback_mask)) >= MIN_MASK_SAMPLES:
            return base.rmse(values[fallback_mask])
        return base.rmse(values)
    return base.rmse(values[mask])


def mask_count(mask: np.ndarray) -> float:
    return float(np.count_nonzero(mask))


def require_finite(label: str, *arrays: np.ndarray) -> None:
    for array in arrays:
        if not np.all(np.isfinite(array)):
            raise base.TrialFailed(f"non-finite values in {label}")


def calculate_track_metrics(
    data: dict[str, np.ndarray],
    duration_s: float,
    speedup: float,
    score_mode: str,
) -> dict[str, float]:
    for key in ["real_pose", "real_vel", "sim_pose", "sim_vel"]:
        if data[key].shape[0] < 3:
            raise base.TrialFailed(f"not enough samples for {key}: {data[key].shape[0]}")

    start = max(
        float(data["real_pose"][0, 0]),
        float(data["real_vel"][0, 0]),
        float(data["sim_pose"][0, 0]),
        float(data["sim_vel"][0, 0]),
    )
    end = min(
        float(data["real_pose"][-1, 0]),
        float(data["real_vel"][-1, 0]),
        float(data["sim_pose"][-1, 0]),
        float(data["sim_vel"][-1, 0]),
        duration_s,
    )
    wall_overlap_s = end - start
    effective_overlap_s = wall_overlap_s * speedup
    if effective_overlap_s < MIN_OVERLAP_S:
        raise base.TrialFailed(f"only {effective_overlap_s:.2f}s effective overlapping data")

    n = int(wall_overlap_s * SAMPLE_RATE_HZ)
    if n < base.MIN_SAMPLES_FOR_SCORE:
        raise base.TrialFailed(f"only {n} score samples")
    t = np.linspace(start, end, n)
    t_effective = (t - t[0]) * speedup

    real_x = interpolate_series(data["real_pose"], t, 1)
    real_y = interpolate_series(data["real_pose"], t, 2)
    real_yaw = interpolate_series(data["real_pose"], t, 3, unwrap=True)
    real_vx = interpolate_series(data["real_vel"], t, 1)
    real_yawrate = interpolate_series(data["real_vel"], t, 2)

    sim_x = interpolate_series(data["sim_pose"], t, 1)
    sim_y = interpolate_series(data["sim_pose"], t, 2)
    sim_yaw = interpolate_series(data["sim_pose"], t, 3, unwrap=True)
    sim_vx = interpolate_series(data["sim_vel"], t, 1)
    sim_yawrate = interpolate_series(data["sim_vel"], t, 2)

    require_finite(
        "interpolated state",
        real_x,
        real_y,
        real_yaw,
        real_vx,
        real_yawrate,
        sim_x,
        sim_y,
        sim_yaw,
        sim_vx,
        sim_yawrate,
    )

    if data["commands"].shape[0] >= 2:
        cmd_throttle = interpolate_series(data["commands"], t, 1)
        cmd_steer = interpolate_series(data["commands"], t, 2)
    else:
        cmd_throttle = np.zeros_like(t)
        cmd_steer = np.zeros_like(t)
    require_finite("interpolated commands", cmd_throttle, cmd_steer)

    if base.ALIGN_TRAJECTORY_ORIGIN:
        real_x = real_x - real_x[0]
        real_y = real_y - real_y[0]
        sim_x = sim_x - sim_x[0]
        sim_y = sim_y - sim_y[0]
        real_yaw = real_yaw - real_yaw[0]
        sim_yaw = sim_yaw - sim_yaw[0]

    ex = sim_x - real_x
    ey = sim_y - real_y
    eyaw = base.angle_diff(sim_yaw, real_yaw)
    evx = sim_vx - real_vx
    eyawrate = sim_yawrate - real_yawrate
    position_error = np.sqrt(ex * ex + ey * ey)

    real_accel = np.gradient(real_vx, t_effective)
    sim_accel = np.gradient(sim_vx, t_effective)
    eaccel = sim_accel - real_accel
    require_finite("derived errors", ex, ey, eyaw, evx, eyawrate, position_error, eaccel)

    moving = real_vx > MIN_VX_FOR_DYNAMIC_MASKS_MPS
    straight = (
        moving
        & (np.abs(real_yawrate) < STRAIGHT_YAWRATE_MAX_RAD_S)
        & (np.abs(cmd_steer) < STRAIGHT_STEER_MAX_RAD)
    )
    curve = (
        moving
        & (
            (np.abs(real_yawrate) > CURVE_YAWRATE_MIN_RAD_S)
            | (np.abs(cmd_steer) > CURVE_STEER_MIN_RAD)
        )
    )
    accel = straight & (cmd_throttle > ACCEL_THROTTLE_MIN)
    coast = moving & (np.abs(cmd_throttle) < COAST_THROTTLE_MAX)
    powered_curve = curve & (np.abs(cmd_throttle) > DIFF_THROTTLE_MIN)

    metrics = {
        "x_rmse": base.rmse(ex),
        "y_rmse": base.rmse(ey),
        "position_rmse": base.rmse(position_error),
        "position_max": float(np.max(position_error)),
        "yaw_rmse": base.rmse(eyaw),
        "yaw_max": float(np.max(np.abs(eyaw))),
        "yawrate_rmse": base.rmse(eyawrate),
        "yawrate_max": float(np.max(np.abs(eyawrate))),
        "vx_rmse": base.rmse(evx),
        "vx_max": float(np.max(np.abs(evx))),
        "accel_rmse": base.rmse(eaccel),
        "straight_vx_rmse": masked_rmse(evx, straight),
        "straight_accel_rmse": masked_rmse(eaccel, accel, straight),
        "coast_vx_rmse": masked_rmse(evx, coast, straight),
        "curve_yawrate_rmse": masked_rmse(eyawrate, curve),
        "curve_yaw_rmse": masked_rmse(eyaw, curve),
        "curve_position_rmse": masked_rmse(position_error, curve),
        "powered_curve_vx_rmse": masked_rmse(evx, powered_curve, curve),
        "powered_curve_yawrate_rmse": masked_rmse(eyawrate, powered_curve, curve),
        "overlap_s": float(effective_overlap_s),
        "wall_overlap_s": float(wall_overlap_s),
        "samples": float(n),
        "straight_samples": mask_count(straight),
        "curve_samples": mask_count(curve),
        "accel_samples": mask_count(accel),
        "coast_samples": mask_count(coast),
        "powered_curve_samples": mask_count(powered_curve),
        "real_pose_samples": float(data["real_pose"].shape[0]),
        "sim_pose_samples": float(data["sim_pose"].shape[0]),
        "real_velocity_samples": float(data["real_vel"].shape[0]),
        "sim_velocity_samples": float(data["sim_vel"].shape[0]),
        "command_samples": float(data["commands"].shape[0]),
    }

    if score_mode == "drivetrain":
        score = (
            2.4 * metrics["straight_vx_rmse"]
            + 1.1 * metrics["straight_accel_rmse"]
            + 1.0 * metrics["coast_vx_rmse"]
            + 0.25 * metrics["vx_max"]
        )
    elif score_mode == "steering":
        score = (
            2.8 * metrics["curve_yawrate_rmse"]
            + 1.1 * metrics["curve_yaw_rmse"]
            + 0.25 * metrics["curve_position_rmse"]
            + 0.15 * metrics["yawrate_max"]
        )
    elif score_mode == "differential":
        score = (
            1.6 * metrics["powered_curve_vx_rmse"]
            + 2.2 * metrics["powered_curve_yawrate_rmse"]
            + 0.35 * metrics["curve_position_rmse"]
        )
    elif score_mode == "tire":
        score = (
            1.4 * metrics["curve_position_rmse"]
            + 1.8 * metrics["curve_yawrate_rmse"]
            + 0.8 * metrics["curve_yaw_rmse"]
            + 0.9 * metrics["position_rmse"]
            + 0.8 * metrics["position_max"]
            + 0.35 * metrics["vx_rmse"]
        )
    else:
        score = (
            2.0 * metrics["x_rmse"]
            + 2.0 * metrics["y_rmse"]
            + 1.2 * metrics["position_max"]
            + 0.7 * metrics["vx_rmse"]
            + 0.6 * metrics["yaw_rmse"]
            + 0.8 * metrics["yawrate_rmse"]
        )

    for metric_name, weight in GLOBAL_GUARD_SCORE_WEIGHTS.get(score_mode, {}).items():
        score += weight * metrics[metric_name]

    metrics["score"] = float(score)
    if not all(math.isfinite(value) for value in metrics.values()):
        raise base.TrialFailed("non-finite metric produced by this candidate")
    return metrics


def should_early_stop(
    collector: TrackAwareCollector,
    score_until_s: float,
    speedup: float,
    score_mode: str,
) -> tuple[bool, str]:
    if not ENABLE_EARLY_STOP:
        return False, ""
    effective_elapsed = collector.elapsed() * speedup
    if effective_elapsed < EARLY_STOP_AFTER_EFFECTIVE_S:
        return False, ""
    try:
        metrics = calculate_track_metrics(collector.snapshot(), score_until_s, speedup, score_mode)
    except base.TrialFailed as exc:
        if "non-finite" in str(exc):
            return True, str(exc)
        return False, ""

    reasons = []
    if metrics["position_max"] > EARLY_STOP_POSITION_MAX_M:
        reasons.append(f"position_max={metrics['position_max']:.2f}m")
    if metrics["vx_max"] > EARLY_STOP_VX_MAX_MPS:
        reasons.append(f"vx_max={metrics['vx_max']:.2f}m/s")
    if metrics["yawrate_max"] > EARLY_STOP_YAWRATE_MAX_RAD_S:
        reasons.append(f"yawrate_max={metrics['yawrate_max']:.2f}rad/s")
    return bool(reasons), ", ".join(reasons)


# =============================================================================
# Trial execution
# =============================================================================


def run_phase_trial(
    trial: int,
    phase: str,
    phase_trial: int,
    score_mode: str,
    params: dict[str, float],
    all_params: list[dict[str, Any]],
    duration_s: float,
    speedup: float,
    collector: TrackAwareCollector,
) -> PhaseTrialResult:
    trial_dir = RESULTS_DIR / f"trial_{trial:04d}_{phase}"
    trial_dir.mkdir(parents=True, exist_ok=True)

    base.apply_params(params, all_params)
    base.bump_state_publish_frequencies()
    base.set_simulation_speedup(speedup)
    wall_duration_s = duration_s / speedup

    collector.reset()
    sim_proc: subprocess.Popen[str] | None = None
    bag_proc: subprocess.Popen[str] | None = None
    early_stop_reason = ""
    score_until_s = 0.0
    try:
        sim_proc = base.start_process(base.simulator_command(), trial_dir / "simulator.log")
        base.wait_for(
            lambda: sim_proc.poll() is not None or (
                collector.counts()["sim_pose"] > 0 and collector.counts()["sim_vel"] > 0
            ),
            base.SIM_READY_TIMEOUT_S,
            "simulator state topics",
        )
        if sim_proc.poll() is not None:
            raise base.TrialFailed(f"simulator exited early with code {sim_proc.returncode}")

        collector.reset()
        bag_proc = base.start_process(
            f"ros2 bag play -s mcap {shlex.quote(str(BAG_PATH))} "
            f"--disable-keyboard-controls --rate {speedup:.6g}",
            trial_dir / "bag.log",
        )
        base.wait_for(
            lambda: bag_proc.poll() is not None or (
                collector.counts()["real_pose"] > 0 and collector.counts()["real_vel"] > 0
            ),
            base.BAG_READY_TIMEOUT_S,
            "bag state topics",
        )
        if bag_proc.poll() is not None:
            raise base.TrialFailed(f"bag play exited early with code {bag_proc.returncode}")

        score_until_s = collector.elapsed() + wall_duration_s
        deadline = time.monotonic() + wall_duration_s
        while time.monotonic() < deadline:
            time.sleep(min(EARLY_STOP_CHECK_PERIOD_WALL_S, max(0.0, deadline - time.monotonic())))
            stop, reason = should_early_stop(collector, score_until_s, speedup, score_mode)
            if stop:
                early_stop_reason = reason
                break

        base.stop_process(bag_proc)
        bag_proc = None
        time.sleep(POST_PLAY_DRAIN_S)

        metrics = calculate_track_metrics(collector.snapshot(), score_until_s, speedup, score_mode)
        metrics["speedup"] = float(speedup)
        metrics["wall_duration_s"] = float(wall_duration_s)
        metrics["simulated_duration_s"] = float(duration_s)
        metrics["early_stopped"] = float(bool(early_stop_reason))
        if early_stop_reason:
            metrics["early_stop_penalty"] = 20.0
            metrics["score"] += metrics["early_stop_penalty"]
            print(base.color(f"Early stop: {early_stop_reason}", "yellow", "bold"))
        else:
            metrics["early_stop_penalty"] = 0.0
        metrics["completed_requested_duration"] = float(not early_stop_reason)

        return PhaseTrialResult(
            trial=trial,
            phase=phase,
            phase_trial=phase_trial,
            score=float(metrics["score"]),
            duration_s=duration_s,
            metrics=metrics,
            params=params.copy(),
        )
    finally:
        base.stop_process(bag_proc)
        base.stop_process(sim_proc)


# =============================================================================
# Results and reporting
# =============================================================================


def write_json(path: Path, data: Any) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write("\n")
    tmp.replace(path)


def append_jsonl(path: Path, data: Any) -> None:
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(data, sort_keys=True) + "\n")


def result_to_dict(result: PhaseTrialResult) -> dict[str, Any]:
    return {
        "trial": result.trial,
        "phase": result.phase,
        "phase_trial": result.phase_trial,
        "score": result.score,
        "duration_s": result.duration_s,
        "metrics": result.metrics,
        "params": result.params,
    }


def next_trial_number(path: Path) -> int:
    if not path.exists():
        return 1
    highest = 0
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            try:
                highest = max(highest, int(json.loads(line).get("trial", 0)))
            except json.JSONDecodeError:
                continue
    return highest + 1


def load_phase_history(
    path: Path,
    phase: str,
    phase_specs: list[dict[str, Any]],
) -> list[PhaseTrialResult]:
    if not RESUME_THIS_SCRIPT_RESULTS or not path.exists():
        return []
    required = {spec["name"] for spec in phase_specs}
    results = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            row = json.loads(line)
            if row.get("failed") or row.get("phase") != phase:
                continue
            score = float(row["score"])
            if not math.isfinite(score):
                continue
            row_params = row.get("params", {})
            if not required.issubset(row_params.keys()):
                continue
            row_metrics = {
                k: float(v)
                for k, v in row["metrics"].items()
                if isinstance(v, (int, float)) and math.isfinite(float(v))
            }
            results.append(PhaseTrialResult(
                trial=int(row["trial"]),
                phase=str(row["phase"]),
                phase_trial=int(row.get("phase_trial", 0)),
                score=score,
                duration_s=float(row["duration_s"]),
                metrics=row_metrics,
                params={k: float(v) for k, v in row_params.items()},
            ))
    return results


def load_saved_accepted_result(path: Path) -> PhaseTrialResult | None:
    if not RESUME_THIS_SCRIPT_RESULTS or not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return None

    row = data.get("last_accepted_result")
    if not isinstance(row, dict):
        return None
    try:
        score = float(row["score"])
    except (KeyError, TypeError, ValueError):
        return None
    if not math.isfinite(score):
        return None
    return PhaseTrialResult(
        trial=int(row.get("trial", 0)),
        phase=str(row.get("phase", "saved_best")),
        phase_trial=int(row.get("phase_trial", 0)),
        score=score,
        duration_s=float(row.get("duration_s", 0.0)),
        metrics={
            k: float(v)
            for k, v in row.get("metrics", {}).items()
            if isinstance(v, (int, float)) and math.isfinite(float(v))
        },
        params={k: float(v) for k, v in row.get("params", {}).items()},
    )


def meaningful_improvement(new_score: float, best_score: float, min_relative: float) -> bool:
    margin = max(abs(best_score) * min_relative, 1e-6)
    return new_score < best_score - margin


def save_current_best(
    params: dict[str, float],
    result: PhaseTrialResult | None,
    certainty: list[dict[str, Any]],
) -> None:
    with (RESULTS_DIR / "best_parameters.yaml").open("w", encoding="utf-8") as f:
        yaml.safe_dump(params, f, sort_keys=False)
    payload: dict[str, Any] = {
        "params": params,
        "certainty_from_current_phase_elite_trials": certainty,
    }
    if result is not None:
        payload["last_accepted_result"] = result_to_dict(result)
    write_json(RESULTS_DIR / "best_result.json", payload)


def print_phase_banner(phase: dict[str, Any], specs: list[dict[str, Any]], duration_s: float, speedup: float) -> None:
    tire_model = tire_model_for_phase(phase)
    print("\n" + base.color("=" * 90, "blue"))
    print(base.color(f"Phase: {phase['name']}", "bold", "green"))
    print(
        f"{base.color('Mode:', 'bold', 'blue')} {phase['score_mode']} | "
        f"{base.color('Trials:', 'bold', 'blue')} {phase['trials']} | "
        f"{base.color('Duration:', 'bold', 'blue')} {duration_s:.1f}s effective "
        f"({duration_s / speedup:.1f}s wall at {speedup:.3g}x) | "
        f"{base.color('Tire:', 'bold', 'blue')} {tire_model} | "
        f"{base.color('Radius:', 'bold', 'blue')} "
        f"{float(phase.get('local_radius', DEFAULT_PHASE_LOCAL_RADIUS)):.3f}"
    )
    print(base.color("Phase parameters:", "bold", "cyan"))
    for spec in specs:
        print(f"  {spec['name']}")
    print(base.color("=" * 90, "blue") + "\n", flush=True)


def print_trial_summary(
    result: PhaseTrialResult,
    phase_best: PhaseTrialResult,
    accepted_result: PhaseTrialResult | None,
    certainty: list[dict[str, Any]],
) -> None:
    m = result.metrics
    print("\n" + base.color("-" * 90, "blue"))
    print(base.color(
        f"Trial {result.trial} | phase={result.phase} #{result.phase_trial} | "
        f"score={result.score:.5f}",
        "bold",
        "green",
    ))
    print(
        base.color("Global pose: ", "bold", "magenta")
        + f"pos_rmse={m['position_rmse']:.3f}m, pos_max={m['position_max']:.3f}m, "
        + f"yaw_rmse={m['yaw_rmse']:.4f}rad, vx_rmse={m['vx_rmse']:.3f}m/s, "
        + f"yawrate_rmse={m['yawrate_rmse']:.4f}rad/s"
    )
    print(
        base.color("Track masks: ", "bold", "blue")
        + f"straight={int(m['straight_samples'])}, curve={int(m['curve_samples'])}, "
        + f"accel={int(m['accel_samples'])}, coast={int(m['coast_samples'])}, "
        + f"powered_curve={int(m['powered_curve_samples'])}"
    )
    print(
        base.color("Phase errors: ", "bold", "yellow")
        + f"straight_vx={m['straight_vx_rmse']:.3f}, accel={m['straight_accel_rmse']:.3f}, "
        + f"curve_yawrate={m['curve_yawrate_rmse']:.4f}, "
        + f"powered_curve_vx={m['powered_curve_vx_rmse']:.3f}"
    )
    print(
        base.color("Duration used: ", "bold", "blue")
        + f"requested={result.duration_s:.1f}s effective, "
        + f"overlap={m['overlap_s']:.1f}s effective "
        + f"({m['wall_overlap_s']:.2f}s wall), "
        + f"early_stop={'yes' if m.get('early_stopped', 0.0) else 'no'}"
    )
    print(base.color(
        f"Best in this phase: trial={phase_best.trial}, score={phase_best.score:.5f}",
        "bold",
        "cyan",
    ))
    if accepted_result is not None:
        print(base.color(
            f"Current accepted fit comes from phase={accepted_result.phase}, "
            f"trial={accepted_result.trial}, phase_score={accepted_result.score:.5f}",
            "bold",
            "cyan",
        ))
    if certainty:
        print(base.color("Most certain current-phase parameters:", "bold", "magenta"))
        for row in certainty[:8]:
            print(
                f"  {base.color(str(row['name']), 'magenta')}: "
                f"mean={float(row['elite_mean']):.6g}, "
                f"std={float(row['elite_std']):.4g}, "
                f"rel_std={float(row['relative_std']):.3f}"
            )
    print(base.color("-" * 90, "blue") + "\n", flush=True)


# =============================================================================
# Main
# =============================================================================


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-trials", type=int, default=MAX_TRIALS)
    parser.add_argument("--duration", type=float, default=INITIAL_EVAL_DURATION_S)
    parser.add_argument("--speedup", type=float, default=SIMULATION_SPEEDUP)
    parser.add_argument("--bag", type=Path, default=BAG_PATH)
    parser.add_argument("--results-dir", type=Path, default=RESULTS_DIR)
    parser.add_argument("--warm-start", type=Path, default=WARM_START_PARAMETERS)
    parser.add_argument("--force-tire-phase", action="store_true", default=FORCE_TIRE_PHASE)
    parser.add_argument(
        "--no-early-stop",
        action="store_true",
        help="Run every trial for the requested duration even if the candidate is already bad.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print config and exit without launching ROS.")
    return parser.parse_args()


def main() -> int:
    global BAG_PATH, RESULTS_DIR, WARM_START_PARAMETERS, ENABLE_EARLY_STOP

    args = parse_args()
    if args.no_early_stop:
        ENABLE_EARLY_STOP = False
    BAG_PATH = args.bag if args.bag.is_absolute() else ROOT / args.bag
    RESULTS_DIR = args.results_dir if args.results_dir.is_absolute() else ROOT / args.results_dir
    WARM_START_PARAMETERS = args.warm_start if args.warm_start.is_absolute() else ROOT / args.warm_start
    configure_base_globals(RESULTS_DIR, BAG_PATH)

    if args.speedup <= 0.0:
        print("--speedup must be > 0", file=sys.stderr)
        return 2
    if not BAG_PATH.exists():
        print(f"Bag not found: {BAG_PATH}", file=sys.stderr)
        return 2

    all_params = all_enabled_params()
    current = load_initial_params(all_params)

    print(base.color("InvictaSim staged Bayesian fitting", "bold", "green"))
    print(f"{base.color('Bag:', 'bold', 'blue')} {BAG_PATH}")
    print(f"{base.color('Results:', 'bold', 'blue')} {RESULTS_DIR}")
    print(f"{base.color('Combined-slip warm start:', 'bold', 'blue')} {WARM_START_PARAMETERS}")
    print(f"{base.color('Staged warm start:', 'bold', 'blue')} {STAGED_WARM_START_PARAMETERS}")
    print(
        f"{base.color('MF6.2 non-tire warm start:', 'bold', 'blue')} "
        f"{MF6_2_NON_TIRE_WARM_START_PARAMETERS} "
        f"({'enabled' if USE_MF6_2_NON_TIRE_WARM_START else 'disabled'})"
    )
    print(f"{base.color('Speedup:', 'bold', 'blue')} {args.speedup:.3g}x")
    print(f"{base.color('Initial tire model:', 'bold', 'blue')} {INITIAL_TIRE_MODEL}")
    print(f"{base.color('Final tire model:', 'bold', 'blue')} {FINAL_TIRE_MODEL}")
    print(f"{base.color('Active parameters:', 'bold', 'blue')} {len(all_params)}")
    print(base.color("Initial accepted parameters:", "bold", "cyan"))
    base.print_params(current)

    if args.dry_run:
        for phase in PHASE_SCHEDULE:
            specs = params_by_name(list(phase["params"]))
            print_phase_banner(phase, specs, args.duration, args.speedup)
        print("Dry run requested; exiting before ROS launch.")
        return 0

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    original_sim_speed = base.read_simulation_speedup()
    accepted_result = load_saved_accepted_result(RESULTS_DIR / "best_result.json")

    def apply_current_on_exit() -> None:
        base.apply_params(current, all_params)
        if RESTORE_SIM_SPEED_ON_EXIT:
            base.set_simulation_speedup(original_sim_speed)

    atexit.register(apply_current_on_exit)

    set_active_tire_model(INITIAL_TIRE_MODEL)
    base.bump_state_publish_frequencies()
    save_current_best(current, accepted_result, [])

    collector, executor, collector_thread = start_collector()
    trial = next_trial_number(RESULTS_DIR / "trials.jsonl")
    remaining_trials = args.max_trials

    try:
        for phase_idx, phase in enumerate(PHASE_SCHEDULE):
            if remaining_trials <= 0:
                break

            required_max = phase.get("requires_position_max_below_m")
            if (
                required_max is not None
                and not args.force_tire_phase
                and accepted_result is not None
                and accepted_result.metrics.get("position_max", math.inf) > float(required_max)
            ):
                print(base.color(
                    f"Skipping {phase['name']} because position_max="
                    f"{accepted_result.metrics['position_max']:.3f}m is above "
                    f"{float(required_max):.3f}m. Use --force-tire-phase to run it anyway.",
                    "yellow",
                    "bold",
                ))
                continue
            if required_max is not None and not args.force_tire_phase and accepted_result is None:
                print(base.color(
                    f"Skipping {phase['name']} because no accepted non-tire result exists yet.",
                    "yellow",
                    "bold",
                ))
                continue

            phase_specs = params_by_name(list(phase["params"]))
            active_tire_model = tire_model_for_phase(phase)
            set_active_tire_model(active_tire_model)
            print_phase_banner(phase, phase_specs, args.duration, args.speedup)

            phase_history = load_phase_history(RESULTS_DIR / "trials.jsonl", str(phase["name"]), phase_specs)
            phase_results = list(phase_history)
            phase_best = min(phase_results, key=lambda r: r.score) if phase_results else None
            if phase_best is not None:
                current = merge_phase_params(current, phase_best.params, phase_specs, all_params)
                if accepted_result is None or phase_best.trial > accepted_result.trial:
                    accepted_result = phase_best

            local_radius = float(phase.get("local_radius", DEFAULT_PHASE_LOCAL_RADIUS))
            optimizer_specs = local_param_specs(phase_specs, current, local_radius)
            base.LOCAL_INITIAL_DESIGN_RADIUS = local_radius
            base.N_INITIAL_RANDOM_TRIALS = int(phase.get("initial_random_trials", DEFAULT_PHASE_INITIAL_RANDOM_TRIALS))
            optimizer = base.SimpleGaussianProcessBO(optimizer_specs, RANDOM_SEED + phase_idx)
            for previous in phase_history:
                if params_within_specs(previous.params, optimizer_specs):
                    optimizer.observe(optimizer_observation_params(previous.params, optimizer_specs), previous.score)

            already_run = len(phase_history)
            phase_trials_remaining = max(0, int(phase["trials"]) - already_run)
            phase_trial_limit = min(phase_trials_remaining, remaining_trials)
            if phase_trial_limit <= 0:
                print(base.color(f"Phase {phase['name']} already has {already_run} trials; skipping.", "yellow"))
                continue

            no_improvement = 0
            min_trials = int(phase.get("min_trials", DEFAULT_PHASE_MIN_TRIALS))
            patience = int(phase.get("patience", DEFAULT_PHASE_PATIENCE_TRIALS))
            min_relative_improvement = float(
                phase.get("min_relative_improvement", DEFAULT_PHASE_MIN_RELATIVE_IMPROVEMENT)
            )

            for local_phase_trial in range(1, phase_trial_limit + 1):
                phase_trial = already_run + local_phase_trial
                candidate_subset = optimizer.suggest(subset(current, optimizer_specs))
                candidate = current.copy()
                candidate.update(candidate_subset)
                candidate = base.clamp_params_to_bounds(candidate, all_params)

                print("\n" + base.color(
                    f"Starting trial {trial} | phase={phase['name']} "
                    f"({phase_trial}/{phase_trial_limit}) for {args.duration:.1f}s at "
                    f"{args.speedup:.3g}x ({args.duration / args.speedup:.1f}s wall)",
                    "bold",
                    "green",
                ))
                print(base.color("Candidate phase parameters:", "bold", "cyan"))
                base.print_params(subset(candidate, phase_specs))

                try:
                    result = run_phase_trial(
                        trial=trial,
                        phase=str(phase["name"]),
                        phase_trial=phase_trial,
                        score_mode=str(phase["score_mode"]),
                        params=candidate,
                        all_params=all_params,
                        duration_s=args.duration,
                        speedup=args.speedup,
                        collector=collector,
                    )
                except base.TrialFailed as exc:
                    failure_score = 1e6
                    print(base.color(f"Trial {trial} failed: {exc}", "red", "bold"), flush=True)
                    optimizer.observe(optimizer_observation_params(candidate, optimizer_specs), failure_score)
                    append_jsonl(RESULTS_DIR / "trials.jsonl", {
                        "trial": trial,
                        "phase": phase["name"],
                        "phase_trial": phase_trial,
                        "failed": True,
                        "error": str(exc),
                        "score": failure_score,
                        "duration_s": args.duration,
                        "params": candidate,
                    })
                    trial += 1
                    remaining_trials -= 1
                    continue

                optimizer.observe(optimizer_observation_params(candidate, optimizer_specs), result.score)
                phase_results.append(result)
                append_jsonl(RESULTS_DIR / "trials.jsonl", result_to_dict(result))

                if phase_best is None or result.score < phase_best.score:
                    meaningful = (
                        phase_best is None
                        or meaningful_improvement(result.score, phase_best.score, min_relative_improvement)
                    )
                    phase_best = result
                    current = base.clamp_params_to_bounds(result.params, all_params)
                    accepted_result = result
                    base.apply_params(current, all_params)
                    no_improvement = 0 if meaningful else no_improvement + 1
                else:
                    no_improvement += 1

                certainty = optimizer.certainty_report(phase_results)  # type: ignore[arg-type]
                save_current_best(current, accepted_result, certainty)
                print_trial_summary(result, phase_best, accepted_result, certainty)

                trial += 1
                remaining_trials -= 1
                if remaining_trials <= 0:
                    break
                if phase_trial >= min_trials and no_improvement >= patience:
                    print(base.color(
                        f"Advancing from {phase['name']} after {no_improvement} trials "
                        f"without a {min_relative_improvement * 100.0:.2f}% improvement.",
                        "yellow",
                        "bold",
                    ))
                    break

            save_current_best(current, accepted_result, optimizer.certainty_report(phase_results))  # type: ignore[arg-type]

        base.apply_params(current, all_params)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted by user. Applying current accepted staged parameters before exit.")
        base.apply_params(current, all_params)
        save_current_best(current, accepted_result, [])
        return 130
    finally:
        executor.shutdown()
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        collector_thread.join(timeout=3.0)


if __name__ == "__main__":
    raise SystemExit(main())
