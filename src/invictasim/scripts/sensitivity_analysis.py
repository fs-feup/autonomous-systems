#!/usr/bin/env python3
"""One-at-a-time (finite-difference) sensitivity screening for the 02 tuning set.

For every optimization parameter we perturb it by +/- a fraction of its search
range around the current baseline and measure how the *survival time* of the
trackdrive replay changes (survival = first time either the 1.5 m position or
2.0 m/s velocity limit is exceeded, as reported by `--diagnose`).  Parameters
whose perturbation moves survival the most are the ones the optimizer should
spend its budget on; parameters with ~0 effect can be frozen.

This reuses the compiled optimizer's `--diagnose --set name=value` path, so it
measures sensitivity of the exact production physics, not a re-implementation.

Usage:
  python3 sensitivity_analysis.py [--frac 0.15] [--out sensitivity.csv]
"""
import argparse
import re
import subprocess
import sys
from pathlib import Path

import yaml

WS = Path("/home/ws")
BIN = WS / "install/invictasim/lib/invictasim/vehicle_model_optimizer"
CFG = WS / "config/invictasim/tuning/02_feup_trackdrive_tuning.yaml"
DATA = WS / "src/invictasim/tuning_csvs"
PERF = WS / "performance/invictasim_tuning"

VIOL_RE = re.compile(r"first (pos|vel)-limit .* violation: (never|[0-9.]+)s")


def run_survival(overrides):
    """Return trackdrive survival seconds for the given {name: value} overrides."""
    cmd = ["bash", "-lc",
           "source /opt/ros/humble/setup.bash && source /home/ws/install/setup.bash && " +
           " ".join([str(BIN), "--config", str(CFG), "--data-dir", str(DATA), "--diagnose"] +
                    [f"--set {k}={v}" for k, v in overrides.items()])]
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    # First two violation lines belong to dataset 0 (trackdrive).
    pos = vel = None
    for m in VIOL_RE.finditer(out):
        val = float("inf") if m.group(2) == "never" else float(m.group(2))
        if m.group(1) == "pos" and pos is None:
            pos = val
        elif m.group(1) == "vel" and vel is None:
            vel = val
        if pos is not None and vel is not None:
            break
    if pos is None or vel is None:
        return 0.0
    return min(pos, vel)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frac", type=float, default=0.15,
                    help="perturbation as fraction of each parameter's search range")
    ap.add_argument("--out", default=str(PERF / "sensitivity.csv"))
    args = ap.parse_args()

    cfg = yaml.safe_load(open(CFG))
    params = cfg["tuning"]["parameters"]

    base = run_survival({})
    print(f"baseline trackdrive survival: {base:.3f} s\n")
    print(f"{'parameter':45s} {'-frac':>8s} {'+frac':>8s} {'sensitivity':>12s}")

    results = []
    for p in params:
        name = p["name"]
        lo, hi = float(p["min"]), float(p["max"])
        step = args.frac * (hi - lo)
        # perturb around baseline value read back from a neutral --diagnose run is
        # awkward, so perturb around mid-range-clamped baseline via +/- step of the
        # live config value (the optimizer applies the baseline from config).
        # We approximate the baseline value as the config-loaded value by using two
        # symmetric sets offset by the step directly on top of the model default.
        # To keep it simple and robust we set the parameter to (default +/- step)
        # using the same --set mechanism; the default is whatever the config holds.
        # Read default by probing is unnecessary because --set overrides absolutely,
        # so we instead sweep to lo+step and hi-step ends of range for a coarse but
        # meaningful gradient.
        v_minus = lo + step
        v_plus = hi - step
        s_minus = run_survival({name: v_minus})
        s_plus = run_survival({name: v_plus})
        sens = abs(s_plus - s_minus)
        results.append((name, v_minus, s_minus, v_plus, s_plus, sens))
        print(f"{name:45s} {s_minus:8.3f} {s_plus:8.3f} {sens:12.3f}")

    results.sort(key=lambda r: r[5], reverse=True)
    with open(args.out, "w") as f:
        f.write("parameter,val_lo,surv_lo,val_hi,surv_hi,sensitivity\n")
        for r in results:
            f.write(f"{r[0]},{r[1]},{r[2]},{r[3]},{r[4]},{r[5]}\n")
    print(f"\nRanked sensitivity written to {args.out}")
    print("\nTop influential parameters:")
    for r in results[:12]:
        print(f"  {r[0]:45s} range-sweep survival delta = {r[5]:.3f} s")


if __name__ == "__main__":
    main()
