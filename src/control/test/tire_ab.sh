#!/bin/bash
# Settles whether the MPC's tyre peak D needs the simulator's 0.8 LMUX/LMUY scaling applied.
#
# The coefficients are documented as fitted to the forces the sim ACTUALLY PRODUCES, which already
# include LMU, so scaling again would double-apply it - and PDX1*LMUX = 2.179*0.8 = 1.743 matches
# the committed tire_D of 1.747 almost exactly. That is an argument, not a measurement, so each
# candidate gets its own rebuild and repeated runs. A lower D makes the MPC believe it has less
# grip than it really has; the failure that prompted this was 12 cones at the baseline setting.
#
# A = committed. B = A with 0.8 applied to both D. C = B plus the halved inverter_tau, i.e. the
# working tree exactly as found, which separates any tyre effect from the actuator-lag change.
#
# Note for anyone editing: do NOT pkill -f on "node_control" while a build is running - that
# string is also the compiler's target directory name and it will kill gcc mid-build.
WS=/home/ws
PY=$WS/src/control/include/solver/supermpc_acados/supermpc_acados.py
OUT=$WS/src/control/test/mpcc_data
SCRATCH=/tmp/claude-1000/-home-ws/7f9690f5-09b3-4ebe-aeee-de085fcf1014/scratchpad
REPEATS=${REPEATS:-5}

cp "$PY" "$SCRATCH/tire_py_backup.py"
restore() { cp "$SCRATCH/tire_py_backup.py" "$PY"; }
trap restore EXIT

# label:lateral_D:D:tau
VARIANTS="A_committed:2.129:1.747:0.2 B_scaled08:1.703:1.398:0.2 C_scaled08_tau01:1.703:1.398:0.1"

for v in $VARIANTS; do
  label=${v%%:*}; r=${v#*:}; latD=${r%%:*}; r=${r#*:}; D=${r%%:*}; tau=${r##*:}
  echo "=== $label  (lateral_D=$latD D=$D tau=$tau) ==="
  restore
  sed -i "s/^tire_lateral_D = .*/tire_lateral_D = ${latD}/" "$PY"
  sed -i "s/^tire_D = .*/tire_D = ${D}/" "$PY"
  sed -i "s/^inverter_tau = .*/inverter_tau = ${tau}/" "$PY"
  grep -E "^tire_lateral_D|^tire_D |^inverter_tau" "$PY"

  ( cd "$WS" && source /opt/ros/humble/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
         --parallel-workers 1 --executor sequential --packages-select control \
         --cmake-args -DCMAKE_BUILD_TYPE=Release ) > "$SCRATCH/build_${label}.log" 2>&1 \
    || { echo "  BUILD FAILED"; tail -15 "$SCRATCH/build_${label}.log"; continue; }

  ( cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash \
    && python3 src/control/test/supermpc_sweep.py --horizon --repeats "$REPEATS" \
         --timeout 75 --max-cones 12 --max-lap-time 50 --max-error 3.0 \
         --label "$label" --out "$OUT/tire_${label}.csv" ) 2>&1 | tee "$SCRATCH/tire_${label}.log"
done
echo "=== done ==="
