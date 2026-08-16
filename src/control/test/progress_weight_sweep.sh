#!/bin/bash
# Restore the design intent: PROGRESS should dominate, not path tracking.
#
# The configuration everything has been tuned around is [progress 0.05, n 30, theta 20] - a
# lateral weight 600x the progress weight. That is a centreline tracker with a progress term
# bolted on, i.e. the exact behaviour ProgressMPC exists to avoid. It got there because a
# diagnostic deliberately turned the controller INTO a tracker to check the Frenet plumbing; it
# was the only thing finishing laps at the time, so it silently became the baseline. In effect
# the heavy lateral weight was propping the car up to mask wheelspin.
#
# Now that the traction cap addresses the wheelspin directly (traced: commanded throttle
# 0.61-0.80 against a ~0.43 traction limit), the crutch should come off. This sweeps the lateral
# weight down while progress goes up, with the cap held fixed.
#
# The metric that matters is whether it stays clean while the LINE becomes free. Tracking error
# is EXPECTED to rise and is NOT a quality measure for this controller - it only matters past
# ~1.5 m, which is where the car is actually off the track. Judge on cones and completion.
#
# Corridor half-width is swept alongside. The geometry matters: the car is off the track past
# about 1.5 m of centreline deviation, but the CAR IS 1.2 m WIDE, so its own half-width consumes
# 0.6 m of that. Usable deviation for the centre of the car is ~0.9 m, and anything near 1.4
# would put the outer wheels past the cones. 0.6-0.9 is the real range.
WS=/home/ws
CFG=$WS/config/control/invictasim.yaml
OUT=$WS/src/control/test/mpcc_data/progress_weights.csv
SCRATCH=/tmp/claude-1000/-home-ws/7f9690f5-09b3-4ebe-aeee-de085fcf1014/scratchpad
REPEATS=${REPEATS:-6}

cp "$CFG" "$SCRATCH/pw_backup.yaml"
trap 'cp "$SCRATCH/pw_backup.yaml" "$CFG"' EXIT

echo "case,repeat,time,cones_hit,avg_track_err,solver_failures,killed,slip_max,slip_frac_over_05" > "$OUT"

# label:progress:n:theta:corridor_half_width   (traction cap held at 0.45 throughout)
CASES="design_w09:0.2:0.05:0.5:0.9 free_w09:0.2:0.5:2:0.9 mild_w09:0.2:2:5:0.9 free_w06:0.2:0.5:2:0.6 mild_w06:0.2:2:5:0.6 tracker_w08:0.05:30:20:0.8"

for c in $CASES; do
  label=${c%%:*}; r=${c#*:}; wp=${r%%:*}; r=${r#*:}; wn=${r%%:*}; r=${r#*:}; wt=${r%%:*}; width=${r##*:}
  cp "$SCRATCH/pw_backup.yaml" "$CFG"
  python3 - "$wp" "$wn" "$wt" "$width" "$CFG" <<'PY'
import io, re, sys
wp, wn, wt, width, path = sys.argv[1:6]
s = io.open(path, encoding="utf-8").read()
s = re.sub(r"  progressmpc_cost_weights: \[[^\]]*\]",
           "  progressmpc_cost_weights: [%s, %s, %s, 6.0, 0.05, 0.3, 2.0, 4.0]" % (wp, wn, wt), s)
s = re.sub(r"  progressmpc_terminal_cost_weights: \[[^\]]*\]",
           "  progressmpc_terminal_cost_weights: [%s, %s, %s]" % (wp, wn, wt), s)
s = re.sub(r"  progressmpc_max_throttle: [0-9.]+", "  progressmpc_max_throttle: 0.45", s)
s = re.sub(r"  progressmpc_corridor_half_width: [0-9.]+",
           "  progressmpc_corridor_half_width: %s" % width, s)
io.open(path, "w", encoding="utf-8").write(s)
PY
  echo "=== $label  progress=$wp n=$wn theta=$wt width=$width  (cap 0.45)"
  grep -E "progressmpc_cost_weights" "$CFG" | head -1

  ( cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash \
    && python3 src/control/test/supermpc_sweep.py --progress --repeats "$REPEATS" \
         --timeout 70 --max-cones 40 --max-lap-time 90 --max-error 8.0 --max-slip-ratio 0.9 \
         --out "$SCRATCH/pw_${label}.csv" ) 2>&1 | grep -E "lap 1:|abandoned"
  echo "    [$(grep -oE 'throttle <= [0-9.]+' /tmp/supermpc_logs/ctrl_progress_default_r1.log 2>/dev/null | tail -1)]"

  python3 - "$label" "$SCRATCH/pw_${label}.csv" "$OUT" <<'PY'
import csv, os, sys
label, src, dst = sys.argv[1:4]
if not os.path.exists(src):
    raise SystemExit
with open(dst, "a") as handle:
    for r in csv.DictReader(open(src)):
        handle.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (
            label, r.get("repeat", ""), r.get("time", ""), r.get("cones_hit", ""),
            r.get("avg_track_err", ""), r.get("solver_failures", ""), r.get("killed", ""),
            r.get("slip_max", ""), r.get("slip_frac_over_05", "")))
PY

  python3 - "$OUT" <<'PY'
import csv, sys
from collections import OrderedDict
rows = list(csv.DictReader(open(sys.argv[1])))
g = OrderedDict()
for r in rows:
    g.setdefault(r["case"], []).append(r)
print("  --- case | clean | completed | best lap | mean err (higher = freer line) ---")
for c, rs in g.items():
    clean = [r for r in rs if r["cones_hit"] == "0" and r["time"] not in ("", "nan", None)]
    done = [r for r in rs if r["time"] not in ("", "nan", None)]
    errs = [float(r["avg_track_err"]) for r in done
            if r["avg_track_err"] not in ("", "nan", None)]
    print("    %-9s %d/%d   %d/%d   %8s   %s" % (
        c, len(clean), len(rs), len(done), len(rs),
        "%.2f" % min(float(r["time"]) for r in clean) if clean else "-",
        "%.3f" % (sum(errs) / len(errs)) if errs else "-"))
PY
done
echo "=== done ==="
