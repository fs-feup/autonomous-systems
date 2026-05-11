#!/usr/bin/env python3
"""
Bayesian optimization loop for fitting InvictaSim without tire-parameter tuning.

This script intentionally reuses bayes_fit_invictasim.py's simulator/bag/metric
pipeline, but removes every tire.* parameter from the Bayesian optimization
space. The tire model can therefore stay fixed, e.g. MF tire 6.2, while the
optimizer fits motor, throttle curve, transmission, steering, and car drag/yaw
parameters.
"""

from __future__ import annotations

from pathlib import Path

import bayes_fit_invictasim as base


ROOT = Path(__file__).resolve().parent
TIRE_MODEL = "pacejka_MF6_2"
VEHICLE_MODEL_CONFIG = ROOT / "config" / "invictasim" / "vehicle_models" / "FSFEUP02.yaml"


def set_tire_model() -> None:
    data = base.load_yaml(VEHICLE_MODEL_CONFIG)
    data.setdefault("vehicle_model", {})["tire_model"] = TIRE_MODEL
    base.save_yaml(VEHICLE_MODEL_CONFIG, data)


def main() -> int:
    base.TIRE_MODEL = TIRE_MODEL
    base.VEHICLE_MODEL_CONFIG = VEHICLE_MODEL_CONFIG
    set_tire_model()
    base.RESULTS_DIR = ROOT / "bo_results" / "invictasim_fit_no_tire"
    base.PARAM_SPACE = [
        spec
        for spec in base.PARAM_SPACE
        if not str(spec["name"]).startswith("tire.")
    ]
    return base.main()


if __name__ == "__main__":
    raise SystemExit(main())
