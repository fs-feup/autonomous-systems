#!/bin/bash
# How much wheel inertia does this model actually need to be solvable?
#
# The graded grid is demonstrably the right mechanism - capping the tail took NaNs from 94 to 14
# per run. But it has not reached a healthy solver, and the last two attempts made it worse:
#
#   geometric 2 -> 323 ms, uniform 2 substeps, newton 5      94 NaN
#   capped 2 -> 64 ms,     uniform 2 substeps, newton 5      14 NaN   <- best
#   capped, per-stage substeps floor 1,        newton 5      77 NaN   (floor 1 halved the fine end)
#   capped, per-stage substeps floor 2,        newton 8      22 NaN
#
# More substeps and more Newton iterations made it WORSE, which says the failure is not
# integration accuracy. The physical reading agrees: the sim shows slip ratio reaching 8.2, so the
# wheels genuinely spin up ~9x road speed. Past the Pacejka peak dFx/dkappa goes small and flips
# sign relative to the stable branch, so Newton has almost no gradient to work with. The
# trajectory being integrated is itself unphysical - no grid fixes that.
#
# So this sweeps the one quantity that sets the stiffness, holding the graded grid fixed, to find
# the smallest inertia that gives a solvable problem. The answer is a measurement, not a
# preference: 0.2/0.4 is physically correct and 2.0/2.0 was the old 10x fudge that made the model
# blind to wheelspin. If something like 3x works, that is a far smaller lie than the old one and
# the model still predicts spin-up qualitatively.
#
# Reported per case: NaN count, solver failures, completion, best lap.
WS=/home/ws
PY=$WS/src/control/include/solver/progressmpc_acados/progressmpc_acados.py
OUT=$WS/src/control/test/mpcc_data/inertia_sweep.csv
SCRATCH=/tmp/claude-1000/-home-ws/7f9690f5-09b3-4ebe-aeee-de085fcf1014/scratchpad
REPEATS=${REPEATS:-3}

cp "$PY" "$SCRATCH/inertia_py_backup2.py"
trap 'cp "$SCRATCH/inertia_py_backup2.py" "$PY"' EXIT

echo "case,repeat,time,cones_hit,solver_failures,killed,nan_count" > "$OUT"

# label:front:rear   true = 0.2/0.4
CASES="true:0.2:0.4 x3:0.6:1.2 x5:1.0:2.0 old:2.0:2.0"

for c in $CASES; do
  label=${c%%:*}; r=${c#*:}; front=${r%%:*}; rear=${r##*:}

  cp "$SCRATCH/inertia_py_backup2.py" "$PY"
  python3 - "$front" "$rear" "$PY" <<'PY'
import io, re, sys
front, rear, path = sys.argv[1:4]
s = io.open(path, encoding="utf-8").read()
for pattern, replacement in [
    (r"^front_wheel_inertia = [0-9.]+", "front_wheel_inertia = %s" % front),
    (r"^rear_wheel_inertia = [0-9.]+", "rear_wheel_inertia = %s" % rear),
]:
    s, n = re.subn(pattern, replacement, s, flags=re.M)
    if n != 1:
        raise SystemExit("pattern matched %d times, expected 1: %s" % (n, pattern))
io.open(path, "w", encoding="utf-8").write(s)
PY
  [ $? -ne 0 ] && { echo "  EDIT FAILED for $label"; continue; }

  echo "=== $label  I_front=$front I_rear=$rear  (rebuilding)"
  ( cd "$WS" && MAKEFLAGS=-j2 colcon build --packages-select control \
      --cmake-args -DCMAKE_BUILD_TYPE=Release ) > "$SCRATCH/inertia_build_${label}.log" 2>&1
  if [ $? -ne 0 ]; then echo "  BUILD FAILED for $label"; continue; fi

  rm -f /tmp/supermpc_logs/ctrl_progress_default_r*.log
  ( cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash \
    && python3 src/control/test/supermpc_sweep.py --progress --repeats "$REPEATS" \
         --timeout 70 --max-cones 40 --max-lap-time 90 --max-error 8.0 --max-slip-ratio 0.9 \
         --out "$SCRATCH/inertia_${label}.csv" ) 2>&1 | grep -E "lap 1:|abandoned|STALE"

  nan=$(grep -hc "NAN_DETECTED" /tmp/supermpc_logs/ctrl_progress_default_r1.log 2>/dev/null || echo 0)
  echo "    NaN in r1: $nan"

  python3 - "$label" "$SCRATCH/inertia_${label}.csv" "$OUT" "$nan" <<'PY'
import csv, os, sys
label, src, dst, nan = sys.argv[1:5]
if not os.path.exists(src):
    raise SystemExit
with open(dst, "a") as handle:
    for r in csv.DictReader(open(src)):
        handle.write("%s,%s,%s,%s,%s,%s,%s\n" % (
            label, r.get("repeat", ""), r.get("time", ""), r.get("cones_hit", ""),
            r.get("solver_failures", ""), r.get("killed", ""), nan))
PY

  python3 - "$OUT" <<'PY'
import csv, sys
from collections import OrderedDict
rows = list(csv.DictReader(open(sys.argv[1])))
g = OrderedDict()
for r in rows:
    g.setdefault(r["case"], []).append(r)
print("  --- case | completed | clean | best lap | mean failures | NaN ---")
for c, rs in g.items():
    done = [r for r in rs if r["time"] not in ("", "nan", None)]
    clean = [r for r in done if r["cones_hit"] == "0"]
    f = [float(r["solver_failures"]) for r in rs
         if r["solver_failures"] not in ("", "nan", None)]
    print("    %-5s %d/%d  %d/%d  %9s  %7s  %s" % (
        c, len(done), len(rs), len(clean), len(rs),
        "%.2f" % min(float(r["time"]) for r in done) if done else "-",
        "%.1f" % (sum(f)/len(f)) if f else "-", rs[0]["nan_count"]))
PY
done
echo "=== done ==="
