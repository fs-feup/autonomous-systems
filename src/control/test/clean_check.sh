#!/bin/bash
# Clean rebuild, then measure SuperMPC and mpczinho back to back on the same binary.
#
# Context: a rebuild made SuperMPC lose the car on every run, and neither tyre-D candidate
# recovered it. mpczinho - which shares none of that solver - also shifted (25.09 -> 24.66 s,
# err 0.138 -> 0.090), so the install used for the earlier 60-run dataset was evidently stale and
# those numbers describe a binary nobody can reproduce. Before drawing any further conclusion the
# build itself has to be above suspicion, hence rm -rf rather than an incremental build.
#
# The tyre constants are left exactly as the working tree has them - this script is about the
# build, not about the tyre question.
WS=/home/ws
SCRATCH=/tmp/claude-1000/-home-ws/7f9690f5-09b3-4ebe-aeee-de085fcf1014/scratchpad
OUT=$WS/src/control/test/mpcc_data

echo "=== tyre constants under test ==="
grep -E "^tire_lateral_D|^tire_D |^inverter_tau" \
  "$WS/src/control/include/solver/supermpc_acados/supermpc_acados.py"

echo "=== clean rebuild ==="
rm -rf "$WS/build/control" "$WS/install/control"
( cd "$WS" && source /opt/ros/humble/setup.bash \
  && CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
       --parallel-workers 1 --executor sequential --packages-select control \
       --cmake-args -DCMAKE_BUILD_TYPE=Release ) > "$SCRATCH/build_clean.log" 2>&1 \
  || { echo "BUILD FAILED"; tail -30 "$SCRATCH/build_clean.log"; exit 1; }
echo "build ok"
grep -E "SUPERMPC_N |SUPERMPC_NX|SUPERMPC_NU|SUPERMPC_NP " \
  "$WS/build/control/control/supermpc/acados/c_generated_code/acados_solver_supermpc.h"

for arm in supermpc mpczinho; do
  echo "=== $arm ==="
  if [ "$arm" = "supermpc" ]; then FLAG=--horizon; LABEL="clean_supermpc"; else FLAG=--diag; LABEL="clean_mpczinho"; fi
  ( cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash \
    && python3 src/control/test/supermpc_sweep.py $FLAG --repeats 4 --timeout 75 \
         --max-cones 12 --max-lap-time 50 --max-error 3.0 \
         --label "$LABEL" --out "$OUT/${LABEL}.csv" ) 2>&1 | tee "$SCRATCH/${LABEL}.log"
done
echo "=== done ==="
