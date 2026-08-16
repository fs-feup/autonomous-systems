#!/bin/bash
# Which limit is actually holding the car back?
#
# With the solver healthy (0-5 failures) and tracking tight (0.26 m), the car still runs a median
# of 6.7 m/s and peaks at 15.7 where the other controllers see ~22. Something is binding, and
# guessing which has been expensive this session. Each case below moves exactly ONE limit and
# nothing else, so the speed response identifies the binding constraint directly:
#
#   lat100   lambda_lat 0.8 -> 1.0    faster => cornering/corner-speed limit binds
#   long100  lambda_long 0.8 -> 1.0   faster => traction budget binds
#   prog50   progress weight 10 -> 50 faster => neither binds, the COST was the limit
#
# The progress case is the one the L1 penalty rework was meant to make safe: constraints are now
# priced as exact penalties, so a large progress weight can no longer buy its way through the
# tyre budget the way it could at envelope_penalty 1e3.
#
# Reported per case: completion, cones, best lap, and the speed the car actually reached - which
# is the quantity under test, not lap time.
WS=/home/ws
CFG=$WS/config/control/invictasim.yaml
OUT=$WS/src/control/test/mpcc_data/limiter_probe.csv
SCRATCH=/tmp/claude-1000/-home-ws/7f9690f5-09b3-4ebe-aeee-de085fcf1014/scratchpad
REPEATS=${REPEATS:-4}

cp "$CFG" "$SCRATCH/probe_backup.yaml"
trap 'cp "$SCRATCH/probe_backup.yaml" "$CFG"' EXIT

echo "case,repeat,time,cones_hit,avg_track_err,solver_failures,killed,vx_median,vx_max" > "$OUT"

# label:lambda_lat:lambda_long:progress_weight
CASES="base:0.80:0.80:10.0 lat100:1.00:0.80:10.0 long100:0.80:1.00:10.0 prog50:0.80:0.80:50.0"

for c in $CASES; do
  label=${c%%:*}; r=${c#*:}; lat=${r%%:*}; r=${r#*:}; lon=${r%%:*}; wp=${r##*:}
  cp "$SCRATCH/probe_backup.yaml" "$CFG"
  python3 - "$lat" "$lon" "$wp" "$CFG" <<'PY'
import io, re, sys
lat, lon, wp, path = sys.argv[1:5]
s = io.open(path, encoding="utf-8").read()
# Verify each key was FOUND, not that the file changed. A case whose values already match the
# config (the baseline case, by definition) changes nothing and is still perfectly valid - an
# earlier version of this guard aborted the whole run on exactly that.
edits = [
    (r"  progressmpc_grip_utilisation_lateral: [0-9.]+",
     "  progressmpc_grip_utilisation_lateral: %s" % lat),
    (r"  progressmpc_grip_utilisation_longitudinal: [0-9.]+",
     "  progressmpc_grip_utilisation_longitudinal: %s" % lon),
    (r"  progressmpc_cost_weights: \[[^,]*,",
     "  progressmpc_cost_weights: [%s," % wp),
]
for pattern, replacement in edits:
    s, count = re.subn(pattern, replacement, s)
    if count != 1:
        raise SystemExit("pattern matched %d times, expected 1: %s" % (count, pattern))
io.open(path, "w", encoding="utf-8").write(s)
PY
  [ $? -ne 0 ] && { echo "  CONFIG WRITE FAILED for $label"; continue; }

  # Clear the controller logs first: they persist across cases, and reading speeds out of a
  # previous case's leftovers would attribute one case's pace to another.
  rm -f /tmp/supermpc_logs/ctrl_progress_default_r*.log

  echo "=== $label  lambda_lat=$lat lambda_long=$lon progress=$wp"
  ( cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash \
    && python3 src/control/test/supermpc_sweep.py --progress --repeats "$REPEATS" \
         --timeout 70 --max-cones 40 --max-lap-time 90 --max-error 8.0 --max-slip-ratio 0.9 \
         --out "$SCRATCH/probe_${label}.csv" ) 2>&1 | grep -E "lap 1:|abandoned|STALE"

  # Speed actually reached, read from the controller's own state log rather than the sweep's
  # velocity column - that column reports values above 60 m/s, which this car cannot do, so it
  # is measuring something other than body speed and must not be used to judge pace.
  speeds=$(grep -ohE "vx=[0-9.]+" /tmp/supermpc_logs/ctrl_progress_default_r*.log 2>/dev/null \
           | cut -d= -f2 | sort -n)
  vmed=$(echo "$speeds" | awk '{a[NR]=$1} END{if(NR)print a[int(NR/2)]; else print "-"}')
  vmax=$(echo "$speeds" | awk '{a[NR]=$1} END{if(NR)print a[NR]; else print "-"}')

  python3 - "$label" "$SCRATCH/probe_${label}.csv" "$OUT" "$vmed" "$vmax" <<'PY'
import csv, os, sys
label, src, dst, vmed, vmax = sys.argv[1:6]
if not os.path.exists(src):
    raise SystemExit
with open(dst, "a") as handle:
    for r in csv.DictReader(open(src)):
        handle.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (
            label, r.get("repeat", ""), r.get("time", ""), r.get("cones_hit", ""),
            r.get("avg_track_err", ""), r.get("solver_failures", ""), r.get("killed", ""),
            vmed, vmax))
PY

  python3 - "$OUT" <<'PY'
import csv, sys
from collections import OrderedDict
rows = list(csv.DictReader(open(sys.argv[1])))
g = OrderedDict()
for r in rows:
    g.setdefault(r["case"], []).append(r)
print("  --- case | clean | completed | best lap | vx median | vx max ---")
for c, rs in g.items():
    clean = [r for r in rs if r["cones_hit"] == "0" and r["time"] not in ("", "nan", None)]
    done = [r for r in rs if r["time"] not in ("", "nan", None)]
    print("    %-9s %d/%d   %d/%d   %8s   %6s   %6s" % (
        c, len(clean), len(rs), len(done), len(rs),
        "%.2f" % min(float(r["time"]) for r in clean) if clean else "-",
        rs[0]["vx_median"], rs[0]["vx_max"]))
PY
done
echo "=== done ==="
