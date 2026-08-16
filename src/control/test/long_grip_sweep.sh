#!/bin/bash
# Compensate for the inflated wheel inertia with the longitudinal grip budget.
#
# Measured, with the slip kill correctly gated on speed (>3 m/s, so it judges wheelspin rather
# than the standing start):
#   true inertia 0.2/0.4, 2 sub-steps  -> 3/3 slip-killed (1.00, 0.60, 0.72)
#   true inertia 0.2/0.4, 4 sub-steps  -> 27-44 solver failures, one 47.8 s / 7-cone lap
#   true inertia 0.2/0.4, N=20, 3 sub  -> 3/3 slip-killed (0.91, 1.00, 1.00)
#
# So correcting the inertia does NOT fix the torque overshoot, and sub-stepping makes things
# worse rather than better. The reason is in the original comment: at true values the wheel-spin
# mode has a sub-millisecond time constant against 33 ms sub-steps. IRK is implicit so it stays
# stable, but it cannot RESOLVE a mode 30x faster than its step - it smooths it. Resolving it
# would need ~1 ms sub-steps, i.e. ~67x the linearisation cost, against a budget where
# linearisation is already 13 ms of a 23 ms solve. Inflated inertia is a necessary approximation,
# not a shortcut that can simply be removed.
#
# What inflated inertia costs is a systematic optimism about traction: heavy wheels do not spin
# in prediction, so the MPC believes all the torque it commands reaches the road. The remedy is
# to bound the longitudinal demand directly - which is what the longitudinal grip budget is for.
# lambda_long 0.85 doubles as the baseline that was lost when the broken gate invalidated it.
WS=/home/ws
PY=$WS/src/control/include/solver/progressmpc_acados/progressmpc_acados.py
CFG=$WS/config/control/invictasim.yaml
OUT=$WS/src/control/test/mpcc_data/long_grip.csv
SCRATCH=/tmp/claude-1000/-home-ws/7f9690f5-09b3-4ebe-aeee-de085fcf1014/scratchpad
REPEATS=${REPEATS:-3}

cp "$CFG" "$SCRATCH/lg_cfg_backup.yaml"
trap 'cp "$SCRATCH/lg_cfg_backup.yaml" "$CFG"' EXIT

# Restore the inflated inertia and the standard integrator, then build ONCE - every case below
# is runtime config, so nothing here needs another rebuild.
sed -i "s/^front_wheel_inertia = .*/front_wheel_inertia = 2.0/" "$PY"
sed -i "s/^rear_wheel_inertia = .*/rear_wheel_inertia = 2.0/" "$PY"
sed -i "s/^    ocp.solver_options.sim_method_num_steps = .*/    ocp.solver_options.sim_method_num_steps = 2/" "$PY"
sed -i "s/^  progressmpc_prediction_horizon_steps: .*/  progressmpc_prediction_horizon_steps: 30/" "$CFG"
cp "$CFG" "$SCRATCH/lg_cfg_backup.yaml"
grep -E "^front_wheel_inertia|^rear_wheel_inertia" "$PY"

( cd "$WS" && source /opt/ros/humble/setup.bash \
  && CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
       --parallel-workers 1 --executor sequential --packages-select control \
       --cmake-args -DCMAKE_BUILD_TYPE=Release ) > "$SCRATCH/build_lg.log" 2>&1 \
  || { echo "BUILD FAILED"; tail -20 "$SCRATCH/build_lg.log"; exit 1; }
echo "build ok"

echo "case,repeat,time,cones_hit,avg_track_err,solver_failures,killed,t_solve_mean" > "$OUT"

# label:lambda_long:lambda_lat:slip_ratio
CASES="long085_base:0.85:0.70:0.15 long070:0.70:0.70:0.15 long060:0.60:0.70:0.15 long050:0.50:0.70:0.15 long040:0.40:0.70:0.15 long060_lat065:0.60:0.65:0.15 long060_sr010:0.60:0.70:0.10"

for c in $CASES; do
  label=${c%%:*}; r=${c#*:}; lon=${r%%:*}; r=${r#*:}; lat=${r%%:*}; sr=${r##*:}
  cp "$SCRATCH/lg_cfg_backup.yaml" "$CFG"
  sed -i "s/^  progressmpc_grip_utilisation_longitudinal: .*/  progressmpc_grip_utilisation_longitudinal: $lon/" "$CFG"
  sed -i "s/^  progressmpc_grip_utilisation_lateral: .*/  progressmpc_grip_utilisation_lateral: $lat/" "$CFG"
  sed -i "s/^  progressmpc_max_slip_ratio: .*/  progressmpc_max_slip_ratio: $sr/" "$CFG"

  echo "=== $label  long=$lon lat=$lat slip_ratio=$sr"
  ( cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash \
    && python3 src/control/test/supermpc_sweep.py --progress --repeats "$REPEATS" \
         --timeout 70 --max-cones 40 --max-lap-time 90 --max-error 8.0 --max-slip-ratio 0.9 \
         --out "$SCRATCH/lg_${label}.csv" ) 2>&1 | grep -E "lap 1:|abandoned"

  python3 - "$label" "$SCRATCH/lg_${label}.csv" "$OUT" <<'PY'
import csv, os, sys
label, src, dst = sys.argv[1:4]
if not os.path.exists(src):
    raise SystemExit
with open(dst, "a") as handle:
    for r in csv.DictReader(open(src)):
        handle.write("%s,%s,%s,%s,%s,%s,%s,%s\n" % (
            label, r.get("repeat", ""), r.get("time", ""), r.get("cones_hit", ""),
            r.get("avg_track_err", ""), r.get("solver_failures", ""), r.get("killed", ""),
            r.get("t_solve_mean", "")))
PY
done
echo "=== done ==="
