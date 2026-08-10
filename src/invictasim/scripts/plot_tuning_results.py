#!/usr/bin/env python3
"""Plot InvictaSim 02 tuning results.

Produces:
  * convergence.png       - best score / position-RMSE / velocity-RMSE vs time and vs evaluations
  * trajectory.png        - sim-vs-real XY path and per-signal time series (from a --diagnose CSV)

Inputs are the artefacts the optimizer writes to performance/invictasim_tuning/:
  * convergence_log.csv   - elapsed_s,evals,tag,score,position_rmse,velocity_rmse
  * diagnostic_0.csv      - per-step sim-vs-real telemetry for the trackdrive replay

The survival time embedded in the packed score is recovered as score/1e6 (seconds),
which is the primary optimization objective.
"""
import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PERF = Path("/home/ws/performance/invictasim_tuning")


def read_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def plot_convergence(log_path, out_path):
    rows = read_csv(log_path)
    if not rows:
        print("empty convergence log")
        return
    t = [float(r["elapsed_s"]) for r in rows]
    ev = [float(r["evals"]) for r in rows]
    score = [float(r["score"]) for r in rows]
    pos = [float(r["position_rmse"]) for r in rows]
    vel = [float(r["velocity_rmse"]) for r in rows]
    # score ~ survival_seconds * 1e6 (weighted); expose approximate survival.
    surv = [s / 1e6 for s in score]

    fig, ax = plt.subplots(2, 2, figsize=(13, 8))
    ax[0, 0].plot(t, surv, "-o", ms=3, color="#2563eb")
    ax[0, 0].set(xlabel="wall time [s]", ylabel="weighted survival [s]",
                 title="Survival vs wall time")
    ax[0, 1].plot(ev, surv, "-o", ms=3, color="#2563eb")
    ax[0, 1].set(xlabel="evaluations", ylabel="weighted survival [s]",
                 title="Survival vs evaluations")
    ax[1, 0].plot(t, pos, "-o", ms=3, color="#dc2626", label="position RMSE [m]")
    ax[1, 0].plot(t, vel, "-o", ms=3, color="#16a34a", label="velocity RMSE [m/s]")
    ax[1, 0].set(xlabel="wall time [s]", ylabel="RMSE", title="Tracking error of incumbent")
    ax[1, 0].legend()
    ax[1, 1].axis("off")
    ax[1, 1].text(0.02, 0.95,
                  f"improvements logged: {len(rows)}\n"
                  f"final weighted survival: {surv[-1]:.2f} s\n"
                  f"final position RMSE: {pos[-1]:.3f} m\n"
                  f"final velocity RMSE: {vel[-1]:.3f} m/s\n"
                  f"total evaluations: {int(ev[-1])}\n"
                  f"wall time: {t[-1]:.0f} s",
                  va="top", family="monospace", fontsize=11)
    for a in ax.flat:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    print("wrote", out_path)


def plot_trajectory(diag_path, out_path):
    rows = read_csv(diag_path)
    if not rows:
        print("empty diagnostic csv")
        return
    t = [float(r["t_rel"]) for r in rows]
    sx = [float(r["sim_x"]) for r in rows]
    sy = [float(r["sim_y"]) for r in rows]
    rx = [float(r["real_x"]) for r in rows]
    ry = [float(r["real_y"]) for r in rows]
    pe = [float(r["pos_err"]) for r in rows]
    ve = [float(r["vel_err"]) for r in rows]
    svx = [float(r["sim_vx"]) for r in rows]
    rvx = [float(r["real_vx"]) for r in rows]

    # Trim to the survived portion (first limit crossing) for a readable path plot.
    surv_n = len(rows)
    for i, (p, v) in enumerate(zip(pe, ve)):
        if p > 1.5 or v > 2.0:
            surv_n = i
            break

    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    ax[0, 0].plot(rx[:surv_n], ry[:surv_n], color="#111", lw=2, label="real")
    ax[0, 0].plot(sx[:surv_n], sy[:surv_n], color="#2563eb", lw=1.5, ls="--", label="sim")
    ax[0, 0].set(xlabel="x [m]", ylabel="y [m]",
                 title=f"Path (survived {t[surv_n-1] if surv_n else 0:.1f}s)")
    ax[0, 0].axis("equal")
    ax[0, 0].legend()
    ax[0, 1].plot(t, pe, color="#dc2626")
    ax[0, 1].axhline(1.5, color="k", ls=":", label="1.5 m limit")
    ax[0, 1].set(xlabel="t [s]", ylabel="position error [m]", title="Position error")
    ax[0, 1].legend()
    ax[1, 0].plot(t, ve, color="#16a34a")
    ax[1, 0].axhline(2.0, color="k", ls=":", label="2.0 m/s limit")
    ax[1, 0].set(xlabel="t [s]", ylabel="velocity error [m/s]", title="Velocity error")
    ax[1, 0].legend()
    ax[1, 1].plot(t, rvx, color="#111", label="real vx")
    ax[1, 1].plot(t, svx, color="#2563eb", ls="--", label="sim vx")
    ax[1, 1].set(xlabel="t [s]", ylabel="vx [m/s]", title="Longitudinal velocity")
    ax[1, 1].legend()
    for a in ax.flat:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    print("wrote", out_path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default=str(PERF / "convergence_log.csv"))
    ap.add_argument("--diag", default=str(PERF / "diagnostic_0.csv"))
    ap.add_argument("--outdir", default=str(PERF))
    args = ap.parse_args()
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)
    if Path(args.log).exists():
        plot_convergence(args.log, out / "convergence.png")
    if Path(args.diag).exists():
        plot_trajectory(args.diag, out / "trajectory.png")


if __name__ == "__main__":
    main()
