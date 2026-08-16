#!/bin/bash
# Get the wheel inertia RIGHT, and keep the solver usable with it.
#
# The simulator uses 0.2 / 0.4 kg m^2; the MPC model carried 2.0 / 2.0. With the heavy values the
# optimizer predicts the wheels spinning up slowly, so it commands torque the light real wheels
# convert straight into slip: measured, the baseline abandoned 3/3 runs at slip ratio 0.70-1.00
# with ZERO solver failures - solving cleanly while the real wheels were at 100% slip.
#
# The reason it was inflated is real though: at true values the wheel-spin mode settles in well
# under a millisecond against a ~67 ms shooting interval. IRK is implicit so it will not go
# unstable, but a 2-stage Gauss scheme is A-stable rather than L-stable, so a mode that fast is
# damped inaccurately rather than resolved - hence the extra sub-steps in the variants below.
#
# Cost matters here: linearisation is already 13 ms of a 23 ms solve, and stiffer dynamics make
# that worse, so fewer stages are traded against more sub-steps. Solve time is recorded per
# variant, not just lap time.
#
# label:front:rear:num_steps:horizon_steps
WS=/home/ws
PY=$WS/src/control/include/solver/progressmpc_acados/progressmpc_acados.py
CFG=$WS/config/control/invictasim.yaml
OUT=$WS/src/control/test/mpcc_data/inertia_ab.csv
SCRATCH=/tmp/claude-1000/-home-ws/7f9690f5-09b3-4ebe-aeee-de085fcf1014/scratchpad
REPEATS=${REPEATS:-3}

cp "$PY" "$SCRATCH/inertia_py_backup.py"
cp "$CFG" "$SCRATCH/inertia_cfg_backup.yaml"
restore() { cp "$SCRATCH/inertia_py_backup.py" "$PY"; cp "$SCRATCH/inertia_cfg_backup.yaml" "$CFG"; }
trap restore EXIT

echo "case,repeat,time,cones_hit,avg_track_err,solver_failures,killed,t_solve_mean" > "$OUT"

VARIANTS="B_real_n30_s2:0.2:0.4:2:30 C_real_n30_s4:0.2:0.4:4:30 E_real_n20_s3:0.2:0.4:3:20 D_middle_n30_s2:0.6:0.8:2:30"

for v in $VARIANTS; do
  label=${v%%:*}; r=${v#*:}
  fi=${r%%:*}; r=${r#*:}
  ri=${r%%:*}; r=${r#*:}
  steps=${r%%:*}; nstages=${r##*:}

  echo "=== $label  front=$fi rear=$ri substeps=$steps N=$nstages"
  restore
  sed -i "s/^front_wheel_inertia = .*/front_wheel_inertia = ${fi}/" "$PY"
  sed -i "s/^rear_wheel_inertia = .*/rear_wheel_inertia = ${ri}/" "$PY"
  sed -i "s/^    ocp.solver_options.sim_method_num_steps = .*/    ocp.solver_options.sim_method_num_steps = ${steps}/" "$PY"
  sed -i "s/^  progressmpc_prediction_horizon_steps: .*/  progressmpc_prediction_horizon_steps: ${nstages}/" "$CFG"
  grep -E "^front_wheel_inertia|^rear_wheel_inertia" "$PY"

  ( cd "$WS" && source /opt/ros/humble/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
         --parallel-workers 1 --executor sequential --packages-select control \
         --cmake-args -DCMAKE_BUILD_TYPE=Release ) > "$SCRATCH/build_${label}.log" 2>&1 \
    || { echo "  BUILD FAILED"; tail -15 "$SCRATCH/build_${label}.log"; continue; }

  ( cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash \
    && python3 src/control/test/supermpc_sweep.py --progress --repeats "$REPEATS" \
         --timeout 70 --max-cones 40 --max-lap-time 90 --max-error 8.0 --max-slip-ratio 0.5 \
         --out "$SCRATCH/inertia_${label}.csv" ) 2>&1 | grep -E "lap 1:|abandoned"

  python3 - "$label" "$SCRATCH/inertia_${label}.csv" "$OUT" <<'PY'
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
