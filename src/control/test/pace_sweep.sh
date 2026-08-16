#!/bin/bash
# Lap-time tuning for ProgressMPC, on the two levers the evidence actually points at.
#
# What the limiter probe established (4 repeats each, force-input model but the physics carries):
#   lambda_lat 0.8 -> 1.0   0/4, every run departed. Real lateral capability is ~13.6 m/s^2
#                           (1.39 g); asking for 17 plans grip that does not exist. HELD AT 0.8.
#   lambda_long 0.8 -> 1.0  no pace change at all. Not the limiter. HELD.
#   progress 10 -> 50       46.8 s -> 35.2 s, but completion 4/4 -> 2/4.
#
# So pace lives in the progress weight, and its cost was reliability. That cost existed because
# the old model could not see wheelspin - it believed all commanded torque reached the road. With
# the true wheel inertia, the graded grid resolving the spin-up mode, and the slip-ratio bound
# restored, the model should now decline the throttle itself. This sweep tests whether raising
# progress is finally cheap.
#
# The corridor is the other lever, and structurally the larger one. Corner speed goes as
# v = sqrt(lambda_lat * a_lat / kappa), and kappa is set by the LINE, not the centreline: a wider
# corridor lets the car straighten the corner and raise the effective radius. Track half-width is
# ~1.5 m and the car is 1.2 m wide, so ~0.9 m is geometrically usable. At 0.5 m this controller is
# a centreline tracker wearing a corridor's clothes.
#
# Judged on clean laps, not best lap: a fast time from 1 run in 6 is noise, and this campaign has
# twice been misled by exactly that.
WS=/home/ws
CFG=$WS/config/control/invictasim.yaml
OUT=$WS/src/control/test/mpcc_data/pace_sweep.csv
SCRATCH=/tmp/claude-1000/-home-ws/7f9690f5-09b3-4ebe-aeee-de085fcf1014/scratchpad
REPEATS=${REPEATS:-5}

cp "$CFG" "$SCRATCH/pace_backup.yaml"
trap 'cp "$SCRATCH/pace_backup.yaml" "$CFG"' EXIT

if [ ! -f "$OUT" ]; then
  echo "case,repeat,time,cones_hit,avg_track_err,solver_failures,killed,t_solve_mean,slip_max,vx_med,vx_max" > "$OUT"
fi

# label:corridor:progress_weight
# Corridor first at the current progress weight to isolate it, then the two together.
CASES="w05p10:0.5:10.0 w07p10:0.7:10.0 w09p10:0.9:10.0 w07p30:0.7:30.0 w09p30:0.9:30.0 w09p60:0.9:60.0"

for c in $CASES; do
  label=${c%%:*}; r=${c#*:}; width=${r%%:*}; wp=${r##*:}

  if cut -d, -f1 "$OUT" | grep -qx "$label"; then
    echo "=== $label (done, skipping)"; continue
  fi

  cp "$SCRATCH/pace_backup.yaml" "$CFG"
  python3 - "$width" "$wp" "$CFG" <<'PY'
import io, re, sys
width, wp, path = sys.argv[1:4]
s = io.open(path, encoding="utf-8").read()
# Verify each key was FOUND, not that the file changed - a case matching the current config is
# still a valid case, and an earlier version of this guard aborted the run on exactly that.
edits = [
    (r"  progressmpc_corridor_half_width: [0-9.]+",
     "  progressmpc_corridor_half_width: %s" % width),
    (r"  progressmpc_cost_weights: \[[^,]*,", "  progressmpc_cost_weights: [%s," % wp),
]
for pattern, replacement in edits:
    s, count = re.subn(pattern, replacement, s)
    if count != 1:
        raise SystemExit("pattern matched %d times, expected 1: %s" % (count, pattern))
io.open(path, "w", encoding="utf-8").write(s)
PY
  [ $? -ne 0 ] && { echo "  CONFIG WRITE FAILED for $label"; continue; }

  # Controller logs persist across cases; reading speeds from a previous case's leftovers would
  # attribute one case's pace to another.
  rm -f /tmp/supermpc_logs/ctrl_progress_default_r*.log

  echo "=== $label  corridor=$width progress=$wp"
  ( cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash \
    && python3 src/control/test/supermpc_sweep.py --progress --repeats "$REPEATS" \
         --timeout 70 --max-cones 40 --max-lap-time 90 --max-error 8.0 --max-slip-ratio 0.9 \
         --out "$SCRATCH/pace_${label}.csv" ) 2>&1 | grep -E "lap 1:|abandoned|STALE"

  # Speed from the controller's own state log. The sweep's velocity column reports values above
  # 60 m/s, which this car cannot reach, so it is not body speed and must not be used for pace.
  speeds=$(grep -ohE "vx=[0-9.]+" /tmp/supermpc_logs/ctrl_progress_default_r*.log 2>/dev/null \
           | cut -d= -f2 | sort -n)
  vmed=$(echo "$speeds" | awk '{a[NR]=$1} END{if(NR)print a[int(NR/2)]; else print "-"}')
  vmax=$(echo "$speeds" | awk '{a[NR]=$1} END{if(NR)print a[NR]; else print "-"}')
  nan=$(grep -hc "NAN_DETECTED" /tmp/supermpc_logs/ctrl_progress_default_r1.log 2>/dev/null || echo 0)
  echo "    NaN in r1: $nan   vx median $vmed  max $vmax"

  python3 - "$label" "$SCRATCH/pace_${label}.csv" "$OUT" "$vmed" "$vmax" <<'PY'
import csv, os, sys
label, src, dst, vmed, vmax = sys.argv[1:6]
if not os.path.exists(src):
    raise SystemExit
with open(dst, "a") as handle:
    for r in csv.DictReader(open(src)):
        handle.write("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (
            label, r.get("repeat", ""), r.get("time", ""), r.get("cones_hit", ""),
            r.get("avg_track_err", ""), r.get("solver_failures", ""), r.get("killed", ""),
            r.get("t_solve_mean", ""), r.get("slip_max", ""), vmed, vmax))
PY

  python3 - "$OUT" <<'PY'
import csv, sys
from collections import OrderedDict
rows = list(csv.DictReader(open(sys.argv[1])))
g = OrderedDict()
for r in rows:
    g.setdefault(r["case"], []).append(r)
print("  --- case | clean | done | best clean | mean solve | vx med ---")
for c, rs in g.items():
    clean = [r for r in rs if r["cones_hit"] == "0" and r["time"] not in ("", "nan", None)]
    done = [r for r in rs if r["time"] not in ("", "nan", None)]
    ts = [float(r["t_solve_mean"]) for r in rs
          if r["t_solve_mean"] not in ("", "nan", None)]
    print("    %-8s %d/%d  %d/%d  %9s  %7s  %6s" % (
        c, len(clean), len(rs), len(done), len(rs),
        "%.2f" % min(float(r["time"]) for r in clean) if clean else "-",
        "%.1f ms" % (sum(ts)/len(ts)) if ts else "-", rs[0]["vx_med"]))
PY
done
echo "=== done ==="
