#!/bin/bash
# Sweeps the SuperMPC prediction horizon, which needs a solver regeneration per variant and so
# cannot live inside supermpc_sweep.py.
#
# Every planner parameter is now exhausted: a_lat trades pace for reliability monotonically, and
# margin and top speed do nothing. What is left is a ~1-in-5 departure rate that no cost weight
# touches. The horizon is the one untested mechanism that changes what the controller can see
# coming rather than how it weighs what it already sees.
#
# The horizon is 1.0 s over 50 steps = 20 ms per step, about 22 m of lookahead at the ~22 m/s the
# car actually reaches. Solve time is already ~20 ms against a 25 ms command period, so raising
# the step COUNT is not shippable and is not tried. Stretching the horizon TIME at fixed N costs
# nothing at runtime - it only coarsens the discretisation from 20 ms toward 40 ms - so that is
# the axis with headroom. h1.5_n30 is the opposite trade: fewer steps to buy back solve time.
# Solve time is recorded per run so the coarser variants can be judged on cost as well as pace.

WS=/home/ws
CFG=$WS/config/control/invictasim.yaml
OUT=$WS/src/control/test/mpcc_data
SCRATCH=/tmp/claude-1000/-home-ws/7f9690f5-09b3-4ebe-aeee-de085fcf1014/scratchpad
REPEATS=${REPEATS:-5}

cp "$CFG" "$SCRATCH/horizon_cfg_backup.yaml"
restore() { cp "$SCRATCH/horizon_cfg_backup.yaml" "$CFG"; }
trap restore EXIT

# label:seconds:steps
VARIANTS="h1.0_n50:1.0:50 h1.5_n50:1.5:50 h2.0_n50:2.0:50 h1.5_n30:1.5:30"

for variant in $VARIANTS; do
  label=${variant%%:*}; rest=${variant#*:}
  secs=${rest%%:*}; steps=${rest##*:}

  echo "=== $label  (${secs}s / ${steps} steps) ==="
  sed -i "s/^  supermpc_prediction_horizon_seconds:.*/  supermpc_prediction_horizon_seconds: ${secs}/" "$CFG"
  sed -i "s/^  supermpc_prediction_horizon_steps:.*/  supermpc_prediction_horizon_steps: ${steps}/" "$CFG"
  grep -E "supermpc_prediction_horizon" "$CFG"

  # Make, not Ninja, and -j2: Ninja ignores MAKEFLAGS and one job per core OOMs this machine.
  ( cd "$WS" && source /opt/ros/humble/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
         --parallel-workers 1 --executor sequential \
         --packages-select control \
         --cmake-args -DCMAKE_BUILD_TYPE=Release ) \
    > "$SCRATCH/build_${label}.log" 2>&1
  if [ $? -ne 0 ]; then
    echo "  BUILD FAILED - see $SCRATCH/build_${label}.log"; tail -20 "$SCRATCH/build_${label}.log"
    continue
  fi

  ( cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash \
    && python3 src/control/test/supermpc_sweep.py --horizon --repeats "$REPEATS" \
         --timeout 75 --max-cones 12 --max-lap-time 50 --max-error 3.0 \
         --label "$label" --out "$OUT/horizon_${label}.csv" ) \
    2>&1 | tee "$SCRATCH/horizon_${label}.log"
done

echo "=== done ==="
