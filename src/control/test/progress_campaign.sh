#!/bin/bash
# Large-sample tuning campaign for ProgressMPC.
#
# Runs SIX repeats per case, not three. At ~2 clean runs in 6, three repeats cannot separate a
# real effect from noise - twice in the previous session a 0/3 was mistaken for a regression and
# code was changed in response. Six is still modest, but it makes a 5/6 vs 2/6 difference
# meaningful, which is the size of effect worth acting on.
#
# One axis moves at a time around the best known point. Everything is runtime config, so no
# rebuild is needed between cases and the whole campaign runs unattended.
#
# Case columns:
#   label  progress  n  theta  lat  long  width  slip_ratio  margin  term_speed
WS=/home/ws
CFG=$WS/config/control/invictasim.yaml
PCFG=$WS/config/planning/invictasim.yaml
OUT=$WS/src/control/test/mpcc_data/progress_campaign.csv
SCRATCH=/tmp/claude-1000/-home-ws/7f9690f5-09b3-4ebe-aeee-de085fcf1014/scratchpad
REPEATS=${REPEATS:-6}
CASES_FILE=${1:-$WS/src/control/test/campaign_cases.txt}

cp "$CFG" "$SCRATCH/camp_ctrl_backup.yaml"
cp "$PCFG" "$SCRATCH/camp_plan_backup.yaml"
trap 'cp "$SCRATCH/camp_ctrl_backup.yaml" "$CFG"; cp "$SCRATCH/camp_plan_backup.yaml" "$PCFG"' EXIT

# Append, never truncate: a long campaign must survive being interrupted and resumed.
if [ ! -f "$OUT" ]; then
  echo "case,repeat,time,cones_hit,avg_track_err,max_track_err,solver_failures,killed,t_solve_mean,slip_max,slip_frac_over_05" > "$OUT"
fi

while read -r entry; do
  case "$entry" in ''|'#'*) continue ;; esac
  set -- $entry
  label=$1; wp=$2; wn=$3; wt=$4; lat=$5; lon=$6; width=$7; sr=$8; margin=$9; term=${10}

  # Already done? Lets an interrupted campaign resume without repeating work.
  if [ -f "$OUT" ] && cut -d, -f1 "$OUT" | grep -qx "$label"; then
    echo "=== $label (already done, skipping)"
    continue
  fi

  cp "$SCRATCH/camp_ctrl_backup.yaml" "$CFG"
  cp "$SCRATCH/camp_plan_backup.yaml" "$PCFG"
  python3 - "$wp" "$wn" "$wt" "$lat" "$lon" "$width" "$sr" "$term" "$CFG" <<'PY'
import io, re, sys
wp, wn, wt, lat, lon, width, sr, term, path = sys.argv[1:10]
s = io.open(path, encoding="utf-8").read()
before = s
s = re.sub(r"  progressmpc_cost_weights: \[[^\]]*\]",
           "  progressmpc_cost_weights: [%s, %s, %s, 6.0, 0.05, 0.3, 2.0, 4.0]" % (wp, wn, wt), s)
s = re.sub(r"  progressmpc_terminal_cost_weights: \[[^\]]*\]",
           "  progressmpc_terminal_cost_weights: [%s, %s, %s]" % (wp, wn, wt), s)
s = re.sub(r"  progressmpc_grip_utilisation_lateral: [0-9.]+",
           "  progressmpc_grip_utilisation_lateral: %s" % lat, s)
s = re.sub(r"  progressmpc_grip_utilisation_longitudinal: [0-9.]+",
           "  progressmpc_grip_utilisation_longitudinal: %s" % lon, s)
s = re.sub(r"  progressmpc_corridor_half_width: [0-9.]+",
           "  progressmpc_corridor_half_width: %s" % width, s)
s = re.sub(r"  progressmpc_max_slip_ratio: [0-9.]+",
           "  progressmpc_max_slip_ratio: %s" % sr, s)
s = re.sub(r"  progressmpc_terminal_speed_factor: [0-9.]+",
           "  progressmpc_terminal_speed_factor: %s" % term, s)
if s == before:
    raise SystemExit("no config key matched")
io.open(path, "w", encoding="utf-8").write(s)
PY
  if [ $? -ne 0 ]; then echo "  CONFIG WRITE FAILED for $label"; continue; fi
  sed -i "s/^  smoothing_safety_margin: .*/  smoothing_safety_margin: $margin/" "$PCFG"

  echo "=== $label  prog=$wp n=$wn th=$wt lat=$lat lon=$lon w=$width sr=$sr margin=$margin term=$term"
  ( cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash \
    && python3 src/control/test/supermpc_sweep.py --progress --repeats "$REPEATS" \
         --timeout 70 --max-cones 40 --max-lap-time 90 --max-error 8.0 --max-slip-ratio 0.9 \
         --out "$SCRATCH/camp_${label}.csv" ) 2>&1 | grep -E "lap 1:|abandoned"

  python3 - "$label" "$SCRATCH/camp_${label}.csv" "$OUT" <<'PY'
import csv, os, sys
label, src, dst = sys.argv[1:4]
if not os.path.exists(src):
    raise SystemExit
with open(dst, "a") as handle:
    for r in csv.DictReader(open(src)):
        handle.write("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (
            label, r.get("repeat", ""), r.get("time", ""), r.get("cones_hit", ""),
            r.get("avg_track_err", ""), r.get("max_track_err", ""),
            r.get("solver_failures", ""), r.get("killed", ""), r.get("t_solve_mean", ""),
            r.get("slip_max", ""), r.get("slip_frac_over_05", "")))
PY
  # Running tally, so the campaign can be read at any point without waiting for it to finish.
  python3 - "$OUT" <<'PY'
import csv, sys
from collections import OrderedDict
rows = list(csv.DictReader(open(sys.argv[1])))
g = OrderedDict()
for r in rows:
    g.setdefault(r["case"], []).append(r)
best = []
for c, rs in g.items():
    clean = [r for r in rs if r["cones_hit"] == "0" and r["time"] not in ("", "nan", None)]
    if clean:
        best.append((len(clean) / len(rs), min(float(r["time"]) for r in clean), c, len(rs)))
best.sort(key=lambda t: (-t[0], t[1]))
print("  --- leaderboard (clean rate, then best lap) ---")
for rate, lap, c, n in best[:5]:
    print("    %-22s %d/%d clean   best %.2f s" % (c, round(rate * n), n, lap))
PY
done < "$CASES_FILE"
echo "=== done ==="
