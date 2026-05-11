#!/usr/bin/env python3
"""
Bayesian optimization loop for fitting InvictaSim FSFEUP02 parameters to a rosbag.

The script launches InvictaSim, plays a bag segment, compares real state-estimation
pose/velocity against simulated state-estimation pose/velocity, scores the trial,
and proposes the next parameter set.

Everything intended for tuning is near the top of this file.
"""

from __future__ import annotations

import argparse
import atexit
import json
import math
import os
import signal
import shlex
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import numpy as np
import rclpy
import yaml
from custom_interfaces.msg import Pose, Velocities
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from scipy.linalg import cho_factor, cho_solve
from scipy.optimize import minimize
from scipy.spatial.distance import cdist
from scipy.special import erf
from scipy.stats import qmc


# ==========================================================f===================
# User-tunable constants
# =============================================================================

ROOT = Path(__file__).resolve().parent
SETUP_BASH = ROOT / "install" / "setup.bash"
#BAG_PATH = ROOT / "bags" / "Zona Industrial 10 laps (second time)(1).mcap"
BAG_PATH = ROOT / "bags" / "DV Trackdrive 5 (Best one yet)(1).mcap"
RESULTS_DIR = ROOT / "bo_results" / "invictasim_fit"
SIMULATION_SPEEDUP = 30.0
TIRE_MODEL = "pacejka_combined_slip"
VEHICLE_MODEL_CONFIG = ROOT / "config" / "invictasim" / "vehicle_models" / "FSFEUP02.yaml"

BAG_TOPICS_TO_PLAY = [
    "/control/command",
    "/state_estimation/velocities",
    "/state_estimation/vehicle_pose",
]

REAL_POSE_TOPIC = "/state_estimation/vehicle_pose"
REAL_VELOCITY_TOPIC = "/state_estimation/velocities"
SIM_POSE_TOPIC = "/invictasim/state_estimation/vehicle_pose"
SIM_VELOCITY_TOPIC = "/invictasim/state_estimation/velocities"

INITIAL_EVAL_DURATION_S = 45.0
MAX_EVAL_DURATION_S = 120.0
DURATION_STEP_S = 10.0
ADVANCE_DURATION_WHEN_MAX_POSITION_ERROR_BELOW_M = 2.0
MIN_TRIALS_BEFORE_DURATION_ADVANCE = 5

MAX_TRIALS = 40000
N_INITIAL_RANDOM_TRIALS = 14
RANDOM_SEED = 7
RESUME_PREVIOUS_RESULTS = False
RESET_RESULTS_DIR_ON_START_WHEN_NOT_RESUMING = True
RESTORE_SIM_SPEED_ON_EXIT = True
ACQUISITION_CANDIDATES = 5000
ACQUISITION_TOP_LOCAL_REFINEMENTS = 8
LOCAL_INITIAL_DESIGN_RADIUS = 0.18
MIN_THROTTLE_CURVE_STEP = 1e-4

# Warm start captured from the first successful invictasim_fit run before
# clearing optimizer history. Values not listed here come from the YAML configs.
INITIAL_PARAMETER_OVERRIDES = {
    "motor.max_peak_torque": 220,
    "motor.max_continous_torque": 120,
    "motor.throttle_torque_curve.0.1": 0.1,
    "motor.throttle_torque_curve.0.2": 0.2,
    "motor.throttle_torque_curve.0.3": 0.3,
    "motor.throttle_torque_curve.0.4": 0.4,
    "motor.throttle_torque_curve.0.5": 0.5,
    "motor.throttle_torque_curve.0.6": 0.6,
    "motor.throttle_torque_curve.0.7": 0.7,
    "motor.throttle_torque_curve.0.8": 0.8,
    "motor.throttle_torque_curve.0.9": 0.9,
    "tire.B_lateral": 9.824863326107298,
    "tire.C_lateral": 1.845490454135323,
    "tire.D_lateral": 2.0300300009992363,
    "tire.E_lateral": 0.8426918839751858,
    "tire.B_longitudinal": 16.022816965259135,
    "tire.C_longitudinal": 1.5617768652829058,
    "tire.D_longitudinal": 1.8370232379248839,
    "tire.E_longitudinal": 0.20615569595109207,
    "tire.relaxation_length": 0.043523594447209726,
    "tire.effective_tire_r": 0.21193366239014186,
    "transmission.efficiency": 0.9676223773701944,
    "transmission.viscous_drag_coeff": 0.015301232289824423,
    "transmission.coulomb_drag": 1.725490376652393,
    "transmission.kv": 24.83963543725823,
    "transmission.t_max": 361.6012656407902,
    "steering_motor.time_constant": 0.04663678866888891,
    "steering.ackerman_factor": 0.1802938465027267,
    "car.front_bearing_drag": 0.0,
    "car.Izz": 133.5828282232209,
}

USE_ROS2_RUN_FOR_SIMULATOR = True
SIM_READY_TIMEOUT_S = 5.0
BAG_READY_TIMEOUT_S = 5.0
POST_PLAY_DRAIN_S = 0.5
PROCESS_TERMINATE_TIMEOUT_S = 3.0

SAMPLE_RATE_HZ = 50.0
MIN_OVERLAP_S = 5.0
MIN_SAMPLES_FOR_SCORE = 50
ALIGN_TRAJECTORY_ORIGIN = True

ERROR_WEIGHTS = {
    "x_rmse": 2.0,
    "y_rmse": 2.0,
    "position_max": 1.5,
    "yaw_rmse": 0.4,
    "vx_rmse": 0.5,
}

ANSI = {
    "reset": "\033[0m",
    "bold": "\033[1m",
    "dim": "\033[2m",
    "red": "\033[31m",
    "green": "\033[32m",
    "yellow": "\033[33m",
    "blue": "\033[34m",
    "magenta": "\033[35m",
    "cyan": "\033[36m",
}

STATE_PUBLISH_FREQUENCIES = {
    "pose": 50,
    "velocities": 50,
}

# Set enabled=False for parameters you want present in the script but frozen.
PARAM_SPACE = [
    {
        "name": "motor.max_peak_torque",
        "file": "config/car/motor_model/02_motor.yaml",
        "keys": ["motor", "max_peak_torque"],
        "bounds": [218.0, 222.0],
        "enabled": False,
    },
    {
        "name": "motor.max_continous_torque",
        "file": "config/car/motor_model/02_motor.yaml",
        "keys": ["motor", "max_continous_torque"],
        "bounds": [118.0, 122.0],
        "enabled": False,
    },
    {
        "name": "motor.throttle_torque_curve.0.1",
        "file": "config/car/motor_model/02_motor.yaml",
        "keys": ["motor", "throttle_torque_curve", 0.1],
        "bounds": [0.0, 0.4],
        "enabled": True,
    },
    {
        "name": "motor.throttle_torque_curve.0.2",
        "file": "config/car/motor_model/02_motor.yaml",
        "keys": ["motor", "throttle_torque_curve", 0.2],
        "bounds": [0.0, 0.6],
        "enabled": True,
    },
    {
        "name": "motor.throttle_torque_curve.0.3",
        "file": "config/car/motor_model/02_motor.yaml",
        "keys": ["motor", "throttle_torque_curve", 0.3],
        "bounds": [0.0, 0.8],
        "enabled": True,
    },
    {
        "name": "motor.throttle_torque_curve.0.4",
        "file": "config/car/motor_model/02_motor.yaml",
        "keys": ["motor", "throttle_torque_curve", 0.4],
        "bounds": [0.0, 1.0],
        "enabled": True,
    },
    {
        "name": "motor.throttle_torque_curve.0.5",
        "file": "config/car/motor_model/02_motor.yaml",
        "keys": ["motor", "throttle_torque_curve", 0.5],
        "bounds": [0.2, 1.0],
        "enabled": True,
    },
    {
        "name": "motor.throttle_torque_curve.0.6",
        "file": "config/car/motor_model/02_motor.yaml",
        "keys": ["motor", "throttle_torque_curve", 0.6],
        "bounds": [0.3, 1.0],
        "enabled": True,
    },
    {
        "name": "motor.throttle_torque_curve.0.7",
        "file": "config/car/motor_model/02_motor.yaml",
        "keys": ["motor", "throttle_torque_curve", 0.7],
        "bounds": [0.4, 1.0],
        "enabled": True,
    },
    {
        "name": "motor.throttle_torque_curve.0.8",
        "file": "config/car/motor_model/02_motor.yaml",
        "keys": ["motor", "throttle_torque_curve", 0.8],
        "bounds": [0.4, 1.0],
        "enabled": True,
    },
    {
        "name": "motor.throttle_torque_curve.0.9",
        "file": "config/car/motor_model/02_motor.yaml",
        "keys": ["motor", "throttle_torque_curve", 0.9],
        "bounds": [0.7, 1.0],
        "enabled": True,
    },
    {
        "name": "tire.B_lateral",
        "file": "config/car/tire_model/02_tire.yaml",
        "keys": ["tire", "B_lateral"],
        "bounds": [4.0, 20.0],
        "enabled": True,
    },
    {
        "name": "tire.C_lateral",
        "file": "config/car/tire_model/02_tire.yaml",
        "keys": ["tire", "C_lateral"],
        "bounds": [0.5, 3.2],
        "enabled": True,
    },
    {
        "name": "tire.D_lateral",
        "file": "config/car/tire_model/02_tire.yaml",
        "keys": ["tire", "D_lateral"],
        "bounds": [0.8, 2.8],
        "enabled": True,
    },
    {
        "name": "tire.E_lateral",
        "file": "config/car/tire_model/02_tire.yaml",
        "keys": ["tire", "E_lateral"],
        "bounds": [-0.5, 1.5],
        "enabled": True,
    },
    {
        "name": "tire.B_longitudinal",
        "file": "config/car/tire_model/02_tire.yaml",
        "keys": ["tire", "B_longitudinal"],
        "bounds": [4.0, 20.0],
        "enabled": True,
    },
    {
        "name": "tire.C_longitudinal",
        "file": "config/car/tire_model/02_tire.yaml",
        "keys": ["tire", "C_longitudinal"],
        "bounds": [0.8, 2.4],
        "enabled": True,
    },
    {
        "name": "tire.D_longitudinal",
        "file": "config/car/tire_model/02_tire.yaml",
        "keys": ["tire", "D_longitudinal"],
        "bounds": [0.8, 3.0],
        "enabled": True,
    },
    {
        "name": "tire.E_longitudinal",
        "file": "config/car/tire_model/02_tire.yaml",
        "keys": ["tire", "E_longitudinal"],
        "bounds": [-0.5, 1.2],
        "enabled": True,
    },
    {
        "name": "tire.relaxation_length",
        "file": "config/car/tire_model/02_tire.yaml",
        "keys": ["tire", "relaxation_length"],
        "bounds": [0.005, 0.6],
        "enabled": True,
    },
    {
        "name": "tire.effective_tire_r",
        "file": "config/car/tire_model/02_tire.yaml",
        "keys": ["tire", "effective_tire_r"],
        "bounds": [0.200, 0.240],
        "enabled": True,
    },
    {
        "name": "transmission.efficiency",
        "file": "config/car/transmission_model/02_transmission.yaml",
        "keys": ["transmission", "efficiency"],
        "bounds": [0.80, 0.99],
        "enabled": True,
    },
    {
        "name": "transmission.viscous_drag_coeff",
        "file": "config/car/transmission_model/02_transmission.yaml",
        "keys": ["transmission", "viscous_drag_coeff"],
        "bounds": [0.0, 2.0],
        "enabled": True,
    },
    {
        "name": "transmission.coulomb_drag",
        "file": "config/car/transmission_model/02_transmission.yaml",
        "keys": ["transmission", "coulomb_drag"],
        "bounds": [0.0, 20.0],
        "enabled": True,
    },
    {
        "name": "transmission.kv",
        "file": "config/car/transmission_model/02_transmission.yaml",
        "keys": ["transmission", "kv"],
        "bounds": [0.0, 120.0],
        "enabled": True,
    },
    {
        "name": "transmission.t_max",
        "file": "config/car/transmission_model/02_transmission.yaml",
        "keys": ["transmission", "t_max"],
        "bounds": [10.0, 600.0],
        "enabled": True,
    },
    {
        "name": "steering_motor.time_constant",
        "file": "config/car/steering_motor_model/02_steering_motor.yaml",
        "keys": ["steering_motor", "time_constant"],
        "bounds": [0.001, 0.55],
        "enabled": True,
    },
    {
        "name": "steering.ackerman_factor",
        "file": "config/car/steering_model/02_steering.yaml",
        "keys": ["steering", "ackerman_factor"],
        "bounds": [0.0, 0.5],
        "enabled": True,
    },
    {
        "name": "car.front_bearing_drag",
        "file": "config/car/02.yaml",
        "keys": ["car", "front_bearing_drag"],
        "bounds": [0.0, 5],
        "enabled": True,
    },
    {
        "name": "car.Izz",
        "file": "config/car/02.yaml",
        "keys": ["car", "Izz"],
        "bounds": [70.0, 180.0],
        "enabled": True,
    },
    {
        "name": "aero.drag_coefficient",
        "file": "config/car/aero_model/02_aero.yaml",
        "keys": ["aero", "drag_coefficient"],
        "bounds": [0.35, 1.2],
        "enabled": False,
    },
]


# =============================================================================
# Data structures
# =============================================================================


@dataclass
class TrialResult:
    trial: int
    score: float
    duration_s: float
    metrics: dict[str, float]
    params: dict[str, float]


class TrialFailed(RuntimeError):
    pass


# =============================================================================
# Config helpers
# =============================================================================


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if data is not None else {}


def save_yaml(path: Path, data: dict[str, Any]) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)
    tmp.replace(path)


def get_nested(data: dict[str, Any], keys: list[Any]) -> Any:
    cur: Any = data
    for key in keys:
        cur = cur[key]
    return cur


def set_nested(data: dict[str, Any], keys: list[Any], value: float) -> None:
    cur: Any = data
    for key in keys[:-1]:
        cur = cur[key]
    cur[keys[-1]] = float(value)


def active_params() -> list[dict[str, Any]]:
    return [p for p in PARAM_SPACE if p.get("enabled", True)]


def read_current_params(params: list[dict[str, Any]]) -> dict[str, float]:
    values = {}
    cache: dict[Path, dict[str, Any]] = {}
    for spec in params:
        path = ROOT / spec["file"]
        if path not in cache:
            cache[path] = load_yaml(path)
        values[spec["name"]] = float(get_nested(cache[path], spec["keys"]))
    return values


def clamp_params_to_bounds(param_values: dict[str, float], params: list[dict[str, Any]]) -> dict[str, float]:
    clamped = {}
    for spec in params:
        name = spec["name"]
        lo, hi = spec["bounds"]
        clamped[name] = float(np.clip(param_values[name], lo, hi))
    return enforce_parameter_constraints(clamped, params)


def enforce_parameter_constraints(
    param_values: dict[str, float],
    params: list[dict[str, Any]],
) -> dict[str, float]:
    constrained = param_values.copy()
    curve_names = [
        f"motor.throttle_torque_curve.{point:.1f}"
        for point in np.arange(0.1, 1.0, 0.1)
    ]
    curve_names = [name for name in curve_names if name in constrained]
    if not curve_names:
        return constrained

    # Shape constraint only: the absolute numeric limits are [0, 1], while each
    # interior point is bounded by its neighbors. Example: the 30% point must be
    # above the 20% point and below the 40% point after projection.
    previous = 0.0
    for idx, name in enumerate(curve_names):
        remaining_edges_to_one = len(curve_names) - idx
        lo = previous + MIN_THROTTLE_CURVE_STEP
        hi = 1.0 - remaining_edges_to_one * MIN_THROTTLE_CURVE_STEP
        value = float(np.clip(constrained[name], lo, hi))
        constrained[name] = value
        previous = constrained[name]
    return constrained


def load_seed_params(params: list[dict[str, Any]]) -> dict[str, float] | None:
    values = read_current_params(params)
    for name, value in INITIAL_PARAMETER_OVERRIDES.items():
        if name in values:
            values[name] = float(value)
    return clamp_params_to_bounds(values, params)


def apply_params(param_values: dict[str, float], params: list[dict[str, Any]]) -> None:
    param_values = clamp_params_to_bounds(param_values, params)
    cache: dict[Path, dict[str, Any]] = {}
    for spec in params:
        path = ROOT / spec["file"]
        if path not in cache:
            cache[path] = load_yaml(path)
        set_nested(cache[path], spec["keys"], param_values[spec["name"]])
    for path, data in cache.items():
        save_yaml(path, data)


def bump_state_publish_frequencies() -> None:
    path = ROOT / "config/invictasim/output/ros.yaml"
    data = load_yaml(path)
    vehicle_state = data.setdefault("publish_frequencies", {}).setdefault("vehicle_state", {})
    for key, value in STATE_PUBLISH_FREQUENCIES.items():
        vehicle_state[key] = int(value)
    save_yaml(path, data)


def set_tire_model() -> None:
    data = load_yaml(VEHICLE_MODEL_CONFIG)
    data.setdefault("vehicle_model", {})["tire_model"] = TIRE_MODEL
    save_yaml(VEHICLE_MODEL_CONFIG, data)


def set_simulation_speedup(speedup: float) -> None:
    path = ROOT / "config/invictasim/global.yaml"
    data = load_yaml(path)
    data.setdefault("invictasim", {})["sim_speed"] = float(speedup)
    save_yaml(path, data)


def read_simulation_speedup() -> float:
    path = ROOT / "config/invictasim/global.yaml"
    data = load_yaml(path)
    return float(data.get("invictasim", {}).get("sim_speed", SIMULATION_SPEEDUP))


# =============================================================================
# Math helpers
# =============================================================================


def angle_diff(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return (a - b + np.pi) % (2.0 * np.pi) - np.pi


def normalize_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rmse(v: np.ndarray) -> float:
    return float(np.sqrt(np.mean(v * v)))


def normal_pdf(x: np.ndarray) -> np.ndarray:
    return np.exp(-0.5 * x * x) / math.sqrt(2.0 * math.pi)


def normal_cdf(x: np.ndarray) -> np.ndarray:
    return 0.5 * (1.0 + erf(x / math.sqrt(2.0)))


# =============================================================================
# Lightweight Bayesian optimizer
# =============================================================================


class SimpleGaussianProcessBO:
    def __init__(self, params: list[dict[str, Any]], seed: int) -> None:
        self.params = params
        self.rng = np.random.default_rng(seed)
        self.bounds = np.array([p["bounds"] for p in params], dtype=float)
        self.dim = len(params)
        self.X: list[np.ndarray] = []
        self.y: list[float] = []

        sampler = qmc.LatinHypercube(d=self.dim, seed=seed)
        self.initial_design = sampler.random(N_INITIAL_RANDOM_TRIALS)
        self.initial_idx = 0

    def to_unit(self, params: dict[str, float]) -> np.ndarray:
        x = np.array([params[p["name"]] for p in self.params], dtype=float)
        return (x - self.bounds[:, 0]) / (self.bounds[:, 1] - self.bounds[:, 0])

    def from_unit(self, x: np.ndarray) -> dict[str, float]:
        x = np.clip(x, 0.0, 1.0)
        values = self.bounds[:, 0] + x * (self.bounds[:, 1] - self.bounds[:, 0])
        return enforce_parameter_constraints(
            {p["name"]: float(v) for p, v in zip(self.params, values)},
            self.params,
        )

    def observe(self, params: dict[str, float], score: float) -> None:
        self.X.append(self.to_unit(params))
        self.y.append(float(score))

    def suggest(self, current_params: dict[str, float]) -> dict[str, float]:
        if not self.X:
            return clamp_params_to_bounds(current_params, self.params)
        if self.initial_idx < len(self.initial_design):
            x = self.initial_design[self.initial_idx]
            self.initial_idx += 1
            center = self.to_unit(current_params)
            local_x = center + (x - 0.5) * (2.0 * LOCAL_INITIAL_DESIGN_RADIUS)
            return self.from_unit(local_x)

        X = np.vstack(self.X)
        y = np.asarray(self.y, dtype=float)
        model = self._fit_gp(X, y)

        sampler = qmc.LatinHypercube(d=self.dim, seed=int(self.rng.integers(1, 2**31 - 1)))
        candidates = sampler.random(ACQUISITION_CANDIDATES)
        ei = self._expected_improvement(candidates, model, float(np.min(y)))

        best_indices = np.argsort(ei)[-ACQUISITION_TOP_LOCAL_REFINEMENTS:]
        best_x = candidates[int(np.argmax(ei))]
        best_ei = float(np.max(ei))

        for idx in best_indices:
            start = candidates[idx]
            result = minimize(
                lambda z: -float(self._expected_improvement(np.asarray(z)[None, :], model, float(np.min(y)))[0]),
                start,
                method="L-BFGS-B",
                bounds=[(0.0, 1.0)] * self.dim,
                options={"maxiter": 80, "ftol": 1e-9},
            )
            if result.success and -float(result.fun) > best_ei:
                best_ei = -float(result.fun)
                best_x = np.asarray(result.x)

        return self.from_unit(best_x)

    def _fit_gp(self, X: np.ndarray, y: np.ndarray) -> dict[str, Any]:
        y_mean = float(np.mean(y))
        y_std = float(np.std(y))
        if y_std < 1e-9:
            y_std = 1.0
        yn = (y - y_mean) / y_std

        def nll(theta: np.ndarray) -> float:
            lengthscale = float(np.exp(theta[0]))
            noise = float(np.exp(theta[1]))
            K = self._kernel(X, X, lengthscale) + (noise + 1e-8) * np.eye(len(X))
            try:
                cf = cho_factor(K, lower=True, check_finite=False)
                alpha = cho_solve(cf, yn, check_finite=False)
                return float(0.5 * yn.dot(alpha) + np.sum(np.log(np.diag(cf[0]))) +
                             0.5 * len(X) * np.log(2.0 * np.pi))
            except np.linalg.LinAlgError:
                return 1e12

        result = minimize(
            nll,
            np.log([0.35, 1e-5]),
            method="L-BFGS-B",
            bounds=[(math.log(0.05), math.log(2.0)), (math.log(1e-8), math.log(0.2))],
            options={"maxiter": 80},
        )
        lengthscale = float(np.exp(result.x[0])) if result.success else 0.35
        noise = float(np.exp(result.x[1])) if result.success else 1e-5
        K = self._kernel(X, X, lengthscale) + (noise + 1e-8) * np.eye(len(X))
        cf = cho_factor(K, lower=True, check_finite=False)
        alpha = cho_solve(cf, yn, check_finite=False)
        return {
            "X": X,
            "y_mean": y_mean,
            "y_std": y_std,
            "lengthscale": lengthscale,
            "cho": cf,
            "alpha": alpha,
        }

    @staticmethod
    def _kernel(a: np.ndarray, b: np.ndarray, lengthscale: float) -> np.ndarray:
        d2 = cdist(a / lengthscale, b / lengthscale, metric="sqeuclidean")
        return np.exp(-0.5 * d2)

    def _predict(self, Xs: np.ndarray, model: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
        Ks = self._kernel(Xs, model["X"], model["lengthscale"])
        mu = Ks.dot(model["alpha"])
        v = cho_solve(model["cho"], Ks.T, check_finite=False)
        var = np.maximum(1.0 - np.sum(Ks * v.T, axis=1), 1e-10)
        return mu * model["y_std"] + model["y_mean"], np.sqrt(var) * model["y_std"]

    def _expected_improvement(self, Xs: np.ndarray, model: dict[str, Any], best_y: float) -> np.ndarray:
        mu, sigma = self._predict(Xs, model)
        sigma = np.maximum(sigma, 1e-12)
        improvement = best_y - mu
        z = improvement / sigma
        return improvement * normal_cdf(z) + sigma * normal_pdf(z)

    def certainty_report(self, results: list[TrialResult]) -> list[dict[str, float | str]]:
        if len(results) < 3:
            return []

        ordered = sorted(results, key=lambda r: r.score)
        elite = ordered[: max(3, min(12, len(ordered) // 3))]
        scores = np.array([r.score for r in elite], dtype=float)
        scale = max(float(np.std(scores)), 1e-6)
        weights = np.exp(-(scores - float(np.min(scores))) / scale)
        weights /= np.sum(weights)

        rows = []
        for spec in self.params:
            name = spec["name"]
            values = np.array([r.params[name] for r in elite], dtype=float)
            mean = float(np.sum(weights * values))
            std = float(np.sqrt(np.sum(weights * (values - mean) ** 2)))
            lo, hi = spec["bounds"]
            rel = std / max(hi - lo, 1e-12)
            rows.append({
                "name": name,
                "elite_mean": mean,
                "elite_std": std,
                "relative_std": rel,
            })
        return sorted(rows, key=lambda row: float(row["relative_std"]))


# =============================================================================
# ROS data collection
# =============================================================================


class StateCollector(Node):
    def __init__(self) -> None:
        super().__init__("invictasim_bayes_fit_collector")
        self.lock = threading.Lock()
        self.real_pose: list[tuple[float, float, float, float]] = []
        self.real_vel: list[tuple[float, float]] = []
        self.sim_pose: list[tuple[float, float, float, float]] = []
        self.sim_vel: list[tuple[float, float]] = []
        self.t0 = time.monotonic()

        self.create_subscription(Pose, REAL_POSE_TOPIC, self._real_pose_cb, 200)
        self.create_subscription(Velocities, REAL_VELOCITY_TOPIC, self._real_vel_cb, 200)
        self.create_subscription(Pose, SIM_POSE_TOPIC, self._sim_pose_cb, 200)
        self.create_subscription(Velocities, SIM_VELOCITY_TOPIC, self._sim_vel_cb, 200)

    def reset(self) -> None:
        with self.lock:
            self.real_pose.clear()
            self.real_vel.clear()
            self.sim_pose.clear()
            self.sim_vel.clear()
            self.t0 = time.monotonic()

    def snapshot(self) -> dict[str, np.ndarray]:
        with self.lock:
            return {
                "real_pose": np.asarray(self.real_pose, dtype=float),
                "real_vel": np.asarray(self.real_vel, dtype=float),
                "sim_pose": np.asarray(self.sim_pose, dtype=float),
                "sim_vel": np.asarray(self.sim_vel, dtype=float),
            }

    def counts(self) -> dict[str, int]:
        with self.lock:
            return {
                "real_pose": len(self.real_pose),
                "real_vel": len(self.real_vel),
                "sim_pose": len(self.sim_pose),
                "sim_vel": len(self.sim_vel),
            }

    def elapsed(self) -> float:
        with self.lock:
            return time.monotonic() - self.t0

    def _stamp(self) -> float:
        return time.monotonic() - self.t0

    def _real_pose_cb(self, msg: Pose) -> None:
        with self.lock:
            self.real_pose.append((self._stamp(), msg.x, msg.y, normalize_angle(msg.theta)))

    def _real_vel_cb(self, msg: Velocities) -> None:
        with self.lock:
            self.real_vel.append((self._stamp(), msg.velocity_x))

    def _sim_pose_cb(self, msg: Pose) -> None:
        with self.lock:
            self.sim_pose.append((self._stamp(), msg.x, msg.y, normalize_angle(msg.theta)))

    def _sim_vel_cb(self, msg: Velocities) -> None:
        with self.lock:
            self.sim_vel.append((self._stamp(), msg.velocity_x))


def start_collector() -> tuple[StateCollector, SingleThreadedExecutor, threading.Thread]:
    if not rclpy.ok():
        rclpy.init(args=None)
    collector = StateCollector()
    executor = SingleThreadedExecutor()
    executor.add_node(collector)
    thread = threading.Thread(target=executor.spin, daemon=False)
    thread.start()
    return collector, executor, thread


def interpolate_series(data: np.ndarray, t: np.ndarray, value_col: int, unwrap: bool = False) -> np.ndarray:
    values = data[:, value_col]
    if unwrap:
        values = np.unwrap(values)
    return np.interp(t, data[:, 0], values)


def calculate_metrics(
    data: dict[str, np.ndarray],
    duration_s: float,
    time_scale: float = 1.0,
) -> dict[str, float]:
    for key in ["real_pose", "real_vel", "sim_pose", "sim_vel"]:
        if data[key].shape[0] < 3:
            raise TrialFailed(f"not enough samples for {key}: {data[key].shape[0]}")

    start = max(float(data["real_pose"][0, 0]), float(data["real_vel"][0, 0]),
                float(data["sim_pose"][0, 0]), float(data["sim_vel"][0, 0]))
    end = min(float(data["real_pose"][-1, 0]), float(data["real_vel"][-1, 0]),
              float(data["sim_pose"][-1, 0]), float(data["sim_vel"][-1, 0]), duration_s)
    wall_overlap_s = end - start
    effective_overlap_s = wall_overlap_s * time_scale
    if effective_overlap_s < MIN_OVERLAP_S:
        raise TrialFailed(f"only {effective_overlap_s:.2f}s effective overlapping data")

    n = int(wall_overlap_s * SAMPLE_RATE_HZ)
    if n < MIN_SAMPLES_FOR_SCORE:
        raise TrialFailed(f"only {n} score samples")
    t = np.linspace(start, end, n)

    real_x = interpolate_series(data["real_pose"], t, 1)
    real_y = interpolate_series(data["real_pose"], t, 2)
    real_yaw = interpolate_series(data["real_pose"], t, 3, unwrap=True)
    real_vx = interpolate_series(data["real_vel"], t, 1)

    sim_x = interpolate_series(data["sim_pose"], t, 1)
    sim_y = interpolate_series(data["sim_pose"], t, 2)
    sim_yaw = interpolate_series(data["sim_pose"], t, 3, unwrap=True)
    sim_vx = interpolate_series(data["sim_vel"], t, 1)

    if ALIGN_TRAJECTORY_ORIGIN:
        real_x = real_x - real_x[0]
        real_y = real_y - real_y[0]
        sim_x = sim_x - sim_x[0]
        sim_y = sim_y - sim_y[0]
        real_yaw = real_yaw - real_yaw[0]
        sim_yaw = sim_yaw - sim_yaw[0]

    ex = sim_x - real_x
    ey = sim_y - real_y
    eyaw = angle_diff(sim_yaw, real_yaw)
    evx = sim_vx - real_vx
    position_error = np.sqrt(ex * ex + ey * ey)

    metrics = {
        "x_rmse": rmse(ex),
        "y_rmse": rmse(ey),
        "position_rmse": rmse(position_error),
        "position_max": float(np.max(position_error)),
        "yaw_rmse": rmse(eyaw),
        "yaw_max": float(np.max(np.abs(eyaw))),
        "vx_rmse": rmse(evx),
        "vx_max": float(np.max(np.abs(evx))),
        "overlap_s": float(effective_overlap_s),
        "wall_overlap_s": float(wall_overlap_s),
        "samples": float(n),
        "real_pose_samples": float(data["real_pose"].shape[0]),
        "sim_pose_samples": float(data["sim_pose"].shape[0]),
        "real_velocity_samples": float(data["real_vel"].shape[0]),
        "sim_velocity_samples": float(data["sim_vel"].shape[0]),
    }
    score = 0.0
    for key, weight in ERROR_WEIGHTS.items():
        score += weight * metrics[key]
    metrics["score"] = float(score)
    return metrics


# =============================================================================
# Process and trial execution
# =============================================================================


def ros_shell_command(command: str) -> str:
    if SETUP_BASH.exists():
        return f"source {SETUP_BASH} && {command}"
    return command


def start_process(command: str, log_path: Path) -> subprocess.Popen[str]:
    log_file = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(
        ros_shell_command(command),
        shell=True,
        executable="/bin/bash",
        cwd=str(ROOT),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )
    proc._bo_log_file = log_file  # type: ignore[attr-defined]
    return proc


def simulator_command() -> str:
    if USE_ROS2_RUN_FOR_SIMULATOR:
        return (
            "SDL_VIDEODRIVER=wayland ros2 run invictasim invictasim "
            "--ros-args -r __ns:=/invictasim -r __node:=invictasim_node"
        )
    return "ros2 launch invictasim invictasim.launch.py"


def wait_for(condition: Callable[[], bool], timeout_s: float, label: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(0.05)
    raise TrialFailed(f"timed out waiting for {label}")


def stop_process(proc: subprocess.Popen[str] | None) -> None:
    if proc is None or proc.poll() is not None:
        if proc is not None and hasattr(proc, "_bo_log_file"):
            proc._bo_log_file.close()  # type: ignore[attr-defined]
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=PROCESS_TERMINATE_TIMEOUT_S)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=PROCESS_TERMINATE_TIMEOUT_S)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
    finally:
        if hasattr(proc, "_bo_log_file"):
            proc._bo_log_file.close()  # type: ignore[attr-defined]


def run_trial(
    trial: int,
    params: dict[str, float],
    duration_s: float,
    speedup: float,
    collector: StateCollector,
) -> TrialResult:
    trial_dir = RESULTS_DIR / f"trial_{trial:04d}"
    trial_dir.mkdir(parents=True, exist_ok=True)
    apply_params(params, active_params())
    bump_state_publish_frequencies()
    set_simulation_speedup(speedup)
    wall_duration_s = duration_s / speedup

    collector.reset()
    sim_proc: subprocess.Popen[str] | None = None
    bag_proc: subprocess.Popen[str] | None = None
    try:
        sim_proc = start_process(simulator_command(), trial_dir / "simulator.log")
        wait_for(
            lambda: sim_proc.poll() is not None or (
                collector.counts()["sim_pose"] > 0 and collector.counts()["sim_vel"] > 0
            ),
            SIM_READY_TIMEOUT_S,
            "simulator state topics",
        )
        if sim_proc.poll() is not None:
            raise TrialFailed(f"simulator exited early with code {sim_proc.returncode}")

        collector.reset()
        bag_proc = start_process(
            f"ros2 bag play -s mcap {shlex.quote(str(BAG_PATH))} "
            f"--disable-keyboard-controls --rate {speedup:.6g}",
            trial_dir / "bag.log",
        )
        wait_for(
            lambda: bag_proc.poll() is not None or (
                collector.counts()["real_pose"] > 0 and collector.counts()["real_vel"] > 0
            ),
            BAG_READY_TIMEOUT_S,
            "bag state topics",
        )
        if bag_proc.poll() is not None:
            raise TrialFailed(f"bag play exited early with code {bag_proc.returncode}")

        score_until_s = collector.elapsed() + wall_duration_s
        time.sleep(wall_duration_s)
        stop_process(bag_proc)
        bag_proc = None
        time.sleep(POST_PLAY_DRAIN_S)

        metrics = calculate_metrics(collector.snapshot(), score_until_s, speedup)
        metrics["speedup"] = float(speedup)
        metrics["wall_duration_s"] = float(wall_duration_s)
        metrics["simulated_duration_s"] = float(duration_s)
        return TrialResult(
            trial=trial,
            score=float(metrics["score"]),
            duration_s=duration_s,
            metrics=metrics,
            params=params.copy(),
        )
    finally:
        stop_process(bag_proc)
        stop_process(sim_proc)


# =============================================================================
# Reporting
# =============================================================================


def color(text: str, *styles: str) -> str:
    prefix = "".join(ANSI[style] for style in styles)
    return f"{prefix}{text}{ANSI['reset']}"


def print_params(params: dict[str, float], indent: str = "  ") -> None:
    for name, value in params.items():
        print(f"{indent}{color(name, 'cyan')}: {color(f'{value:.8g}', 'bold')}")


def print_max_errors(metrics: dict[str, float], indent: str = "  ") -> None:
    print(color("Maximum errors this run:", "bold", "yellow"))
    position_max = f"{metrics['position_max']:.3f} m"
    yaw_max = f"{metrics['yaw_max']:.4f} rad"
    vx_max = f"{metrics['vx_max']:.3f} m/s"
    print(f"{indent}{color('position_max', 'yellow')}: {color(position_max, 'bold')}")
    print(f"{indent}{color('yaw_max', 'yellow')}: {color(yaw_max, 'bold')}")
    print(f"{indent}{color('vx_max', 'yellow')}: {color(vx_max, 'bold')}")


def write_json(path: Path, data: Any) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write("\n")
    tmp.replace(path)


def append_jsonl(path: Path, data: Any) -> None:
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(data, sort_keys=True) + "\n")


def result_to_dict(result: TrialResult) -> dict[str, Any]:
    return {
        "trial": result.trial,
        "score": result.score,
        "duration_s": result.duration_s,
        "metrics": result.metrics,
        "params": result.params,
    }


def load_previous_results(path: Path, params: list[dict[str, Any]]) -> list[TrialResult]:
    if not RESUME_PREVIOUS_RESULTS or not path.exists():
        return []
    required = [spec["name"] for spec in params]
    required_set = set(required)
    results = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            row = json.loads(line)
            if row.get("failed"):
                continue
            row_params = row.get("params", {})
            if not required_set.issubset(row_params.keys()):
                continue
            results.append(TrialResult(
                trial=int(row["trial"]),
                score=float(row["score"]),
                duration_s=float(row["duration_s"]),
                metrics={k: float(v) for k, v in row["metrics"].items()},
                params={name: float(row_params[name]) for name in required},
            ))
    return results


def save_best(result: TrialResult, certainty: list[dict[str, Any]]) -> None:
    write_json(RESULTS_DIR / "best_result.json", {
        **result_to_dict(result),
        "certainty_from_elite_trials": certainty,
    })
    with (RESULTS_DIR / "best_parameters.yaml").open("w", encoding="utf-8") as f:
        yaml.safe_dump(result.params, f, sort_keys=False)


def print_trial_summary(result: TrialResult, best: TrialResult, certainty: list[dict[str, Any]]) -> None:
    m = result.metrics
    print("\n" + color("=" * 88, "blue"))
    print(color(
        f"Trial {result.trial} finished | score={result.score:.5f} | "
        f"duration={result.duration_s:.1f}s",
        "bold",
        "green",
    ))
    print(
        color("RMSE errors: ", "bold", "magenta") +
        f"pos_rmse={m['position_rmse']:.3f}m, "
        f"x_rmse={m['x_rmse']:.3f}m, y_rmse={m['y_rmse']:.3f}m, "
        f"yaw_rmse={m['yaw_rmse']:.4f}rad, vx_rmse={m['vx_rmse']:.3f}m/s"
    )
    print(
        color("Samples: ", "bold", "blue") +
        f"overlap={m['overlap_s']:.2f}s effective ({m.get('wall_overlap_s', m['overlap_s']):.2f}s wall), "
        f"real_pose={int(m['real_pose_samples'])}, "
        f"sim_pose={int(m['sim_pose_samples'])}, real_v={int(m['real_velocity_samples'])}, "
        f"sim_v={int(m['sim_velocity_samples'])}"
    )
    print(color(
        f"Best so far: trial={best.trial}, score={best.score:.5f}, "
        f"max_pos={best.metrics['position_max']:.3f}m",
        "bold",
        "cyan",
    ))
    print(color("Best parameters so far:", "bold", "cyan"))
    print_params(best.params)
    print_max_errors(m)
    if certainty:
        print(color("Most certain parameters from elite-trial covariance:", "bold", "magenta"))
        for row in certainty[:8]:
            print(
                f"  {color(str(row['name']), 'magenta')}: mean={float(row['elite_mean']):.6g}, "
                f"std={float(row['elite_std']):.4g}, rel_std={float(row['relative_std']):.3f}"
            )
    print(color("=" * 88, "blue") + "\n", flush=True)


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
    parser.add_argument("--dry-run", action="store_true", help="Print config and exit without launching ROS.")
    return parser.parse_args()


def main() -> int:
    global BAG_PATH, RESULTS_DIR
    args = parse_args()
    BAG_PATH = args.bag if args.bag.is_absolute() else ROOT / args.bag
    RESULTS_DIR = args.results_dir if args.results_dir.is_absolute() else ROOT / args.results_dir
    if (
        not args.dry_run
        and not RESUME_PREVIOUS_RESULTS
        and RESET_RESULTS_DIR_ON_START_WHEN_NOT_RESUMING
        and RESULTS_DIR.exists()
    ):
        shutil.rmtree(RESULTS_DIR)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    params = active_params()
    if not params:
        print("No enabled parameters in PARAM_SPACE.", file=sys.stderr)
        return 2
    if not BAG_PATH.exists():
        print(f"Bag not found: {BAG_PATH}", file=sys.stderr)
        return 2
    if args.speedup <= 0.0:
        print("--speedup must be > 0", file=sys.stderr)
        return 2

    set_tire_model()
    original_sim_speed = read_simulation_speedup()
    current = read_current_params(params)
    optimizer = SimpleGaussianProcessBO(params, RANDOM_SEED)
    results: list[TrialResult] = load_previous_results(RESULTS_DIR / "trials.jsonl", params)
    seed_params = load_seed_params(params)
    if seed_params is not None:
        current = seed_params
    for previous in results:
        optimizer.observe(previous.params, previous.score)
    best: TrialResult | None = min(results, key=lambda r: r.score) if results else None
    duration_s = args.duration

    def apply_best_on_exit() -> None:
        if best is not None:
            apply_params(best.params, params)
        if RESTORE_SIM_SPEED_ON_EXIT:
            set_simulation_speedup(original_sim_speed)

    atexit.register(apply_best_on_exit)

    print(color("InvictaSim Bayesian fitting", "bold", "green"))
    print(f"{color('Bag:', 'bold', 'blue')} {BAG_PATH}")
    print(f"{color('Results:', 'bold', 'blue')} {RESULTS_DIR}")
    print(f"{color('Speedup:', 'bold', 'blue')} {args.speedup:.3g}x")
    print(f"{color('Active parameters:', 'bold', 'blue')} {len(params)}")
    print(color("History resume disabled; trial 1 uses INITIAL_PARAMETER_OVERRIDES.", "bold", "yellow"))
    print_params(current)
    if results:
        print(color(f"Resumed {len(results)} previous successful trials.", "yellow"))
        print(color(f"Previous best: trial={best.trial}, score={best.score:.5f}", "yellow"))  # type: ignore[union-attr]

    if args.dry_run:
        print("Dry run requested; exiting before ROS launch.")
        return 0

    collector, executor, collector_thread = start_collector()

    try:
        start_trial = (max((r.trial for r in results), default=0) + 1)
        end_trial = start_trial + args.max_trials - 1
        for trial in range(start_trial, end_trial + 1):
            candidate = optimizer.suggest(current if best is None else best.params)
            print("\n" + color(
                f"Starting trial {trial} ({trial - start_trial + 1}/{args.max_trials} this run) "
                f"for {duration_s:.1f}s at {args.speedup:.3g}x "
                f"({duration_s / args.speedup:.1f}s wall)",
                "bold",
                "green",
            ))
            print(color("Candidate parameters:", "bold", "cyan"))
            print_params(candidate)

            try:
                result = run_trial(trial, candidate, duration_s, args.speedup, collector)
            except TrialFailed as exc:
                print(f"Trial {trial} failed: {exc}", flush=True)
                failure_score = 1e6
                optimizer.observe(candidate, failure_score)
                append_jsonl(RESULTS_DIR / "trials.jsonl", {
                    "trial": trial,
                    "failed": True,
                    "error": str(exc),
                    "score": failure_score,
                    "duration_s": duration_s,
                    "params": candidate,
                })
                continue

            optimizer.observe(candidate, result.score)
            results.append(result)
            append_jsonl(RESULTS_DIR / "trials.jsonl", result_to_dict(result))

            if best is None or result.score < best.score:
                best = result
                apply_params(best.params, params)

            certainty = optimizer.certainty_report(results)
            save_best(best, certainty)
            print_trial_summary(result, best, certainty)

            if (len(results) >= MIN_TRIALS_BEFORE_DURATION_ADVANCE and
                    best.metrics["position_max"] < ADVANCE_DURATION_WHEN_MAX_POSITION_ERROR_BELOW_M and
                    duration_s < MAX_EVAL_DURATION_S):
                duration_s = min(MAX_EVAL_DURATION_S, duration_s + DURATION_STEP_S)
                print(f"Position max below threshold; next duration is {duration_s:.1f}s")

        if best is not None:
            apply_params(best.params, params)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted by user. Applying best parameters found so far before exit.")
        if best is not None:
            apply_params(best.params, params)
            certainty = optimizer.certainty_report(results)
            save_best(best, certainty)
            print(f"Best trial: {best.trial}, score={best.score:.5f}")
            print_params(best.params)
        return 130
    finally:
        executor.shutdown()
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        collector_thread.join(timeout=3.0)


if __name__ == "__main__":
    raise SystemExit(main())
