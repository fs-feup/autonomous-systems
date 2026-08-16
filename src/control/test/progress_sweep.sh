#!/bin/bash
# Tuning campaign for ProgressMPC.
#
# Baseline: 25.30 s / 0 cones / 0.179 m on the first clean lap, against mpczinho's 24.68 s /
# 0 cones / 0.090 m on the same binary. Only 1 run in 3 is clean, so the first job is
# consistency, not pace - a fast lap that happens once in three is worth nothing.
#
# The knobs, and why each is in the sweep:
#   slip_ratio  - the constraint that produced the first clean lap at all. Too loose and the
#                 optimizer spins the rears past the force peak; too tight and it cannot
#                 accelerate.
#   lambda      - grip utilisation, the intended smooth pace knob.
#   n weight    - trades line freedom against staying near the centre. Too low and the cost is
#                 flat laterally, which leaves the QP badly conditioned; too high and the
#                 controller degenerates into the centreline tracker this design exists to avoid.
#   progress    - must dominate the regularisers but stay well under the slack penalties.
#   half_width  - the corridor itself.
#
# Everything here is runtime config, so no rebuild is needed between cases.
WS=/home/ws
CFG=$WS/config/control/invictasim.yaml
PCFG=$WS/config/planning/invictasim.yaml
OUT=$WS/src/control/test/mpcc_data/progress_tuning.csv
SCRATCH=/tmp/claude-1000/-home-ws/7f9690f5-09b3-4ebe-aeee-de085fcf1014/scratchpad
REPEATS=${REPEATS:-3}
CASES_FILE=${1:-$WS/src/control/test/progress_cases.txt}

cp "$CFG" "$SCRATCH/ptune_ctrl_backup.yaml"
cp "$PCFG" "$SCRATCH/ptune_plan_backup.yaml"
trap 'cp "$SCRATCH/ptune_ctrl_backup.yaml" "$CFG"; cp "$SCRATCH/ptune_plan_backup.yaml" "$PCFG"' EXIT

if [ ! -f "$OUT" ]; then
  echo "case,repeat,time,cones_hit,avg_track_err,max_track_err,solver_failures,killed,t_solve_mean" > "$OUT"
fi

while read -r entry; do
  case "$entry" in ''|'#'*) continue ;; esac
  label=$(echo "$entry"   | awk '{print $1}')
  wp=$(echo "$entry"      | awk '{print $2}')
  wn=$(echo "$entry"      | awk '{print $3}')
  wt=$(echo "$entry"      | awk '{print $4}')
  lam=$(echo "$entry"     | awk '{print $5}')
  width=$(echo "$entry"   | awk '{print $6}')
  ratio=$(echo "$entry"   | awk '{print $7}')
  margin=$(echo "$entry"  | awk '{print $8}')

  # Always restore first: settings must never accumulate across cases. A previous campaign was
  # invalidated exactly this way, with one case silently inheriting the last one's overrides.
  cp "$SCRATCH/ptune_ctrl_backup.yaml" "$CFG"
  cp "$SCRATCH/ptune_plan_backup.yaml" "$PCFG"
  python3 - "$wp" "$wn" "$wt" "$lam" "$width" "$ratio" "$CFG" <<'PY'
import io, re, sys
wp, wn, wt, lam, width, ratio, path = sys.argv[1:8]
s = io.open(path, encoding="utf-8").read()
s = re.sub(r"  progressmpc_cost_weights: \[[^\]]*\]",
           "  progressmpc_cost_weights: [%s, %s, %s, 6.0, 0.05, 0.3, 2.0, 4.0]" % (wp, wn, wt), s)
s = re.sub(r"  progressmpc_terminal_cost_weights: \[[^\]]*\]",
           "  progressmpc_terminal_cost_weights: [%s, %s, %s]" % (wp, wn, wt), s)
s = re.sub(r"  progressmpc_grip_utilisation: [0-9.]+",
           "  progressmpc_grip_utilisation: %s" % lam, s)
s = re.sub(r"  progressmpc_corridor_half_width: [0-9.]+",
           "  progressmpc_corridor_half_width: %s" % width, s)
s = re.sub(r"  progressmpc_max_slip_ratio: [0-9.]+",
           "  progressmpc_max_slip_ratio: %s" % ratio, s)
io.open(path, "w", encoding="utf-8").write(s)
PY
  sed -i "s/^  smoothing_safety_margin: .*/  smoothing_safety_margin: $margin/" "$PCFG"

  echo "=== $label  progress=$wp n=$wn theta=$wt lambda=$lam width=$width ratio=$ratio margin=$margin"
  ( cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash \
    && python3 src/control/test/supermpc_sweep.py --progress --repeats "$REPEATS" \
         --timeout 70 --max-cones 40 --max-lap-time 90 --max-error 8.0 \
         --out "$SCRATCH/pt_${label}.csv" ) 2>&1 | grep -E "lap 1:|abandoned"

  python3 - "$label" "$SCRATCH/pt_${label}.csv" "$OUT" <<'PY'
import csv, os, sys
label, src, dst = sys.argv[1:4]
if not os.path.exists(src):
    raise SystemExit
with open(dst, "a") as handle:
    for r in csv.DictReader(open(src)):
        handle.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (
            label, r.get("repeat", ""), r.get("time", ""), r.get("cones_hit", ""),
            r.get("avg_track_err", ""), r.get("max_track_err", ""),
            r.get("solver_failures", ""), r.get("killed", ""), r.get("t_solve_mean", "")))
PY
done < "$CASES_FILE"
echo "=== done ==="
