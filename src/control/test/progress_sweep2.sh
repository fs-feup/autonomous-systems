#!/bin/bash
# Round 2 tuning for ProgressMPC, with lateral and longitudinal grip as separate knobs.
#
# Round 1 established that grip utilisation is the dominant axis: at 0.85 the controller was
# clean roughly 1 run in 6, at 0.70 it was 2 of 3 (25.51 / 25.56 s, 0 cones, err 0.175, zero
# solver failures). Everything else moved the result far less. So this round sweeps the two
# budgets independently - the point of splitting them being that cornering pace and straight-line
# push no longer have to move together.
#
# Written as a separate file rather than editing the round 1 script, because bash reads a script
# incrementally and editing one mid-run can corrupt its execution.
#
# Case file columns:
#   label  progress  n  theta  lambda_lat  lambda_long  width  slip_ratio  margin
WS=/home/ws
CFG=$WS/config/control/invictasim.yaml
PCFG=$WS/config/planning/invictasim.yaml
OUT=$WS/src/control/test/mpcc_data/progress_tuning2.csv
SCRATCH=/tmp/claude-1000/-home-ws/7f9690f5-09b3-4ebe-aeee-de085fcf1014/scratchpad
REPEATS=${REPEATS:-3}
CASES_FILE=${1:-$WS/src/control/test/progress_cases2.txt}

cp "$CFG" "$SCRATCH/ptune2_ctrl_backup.yaml"
cp "$PCFG" "$SCRATCH/ptune2_plan_backup.yaml"
trap 'cp "$SCRATCH/ptune2_ctrl_backup.yaml" "$CFG"; cp "$SCRATCH/ptune2_plan_backup.yaml" "$PCFG"' EXIT

if [ ! -f "$OUT" ]; then
  echo "case,repeat,time,cones_hit,avg_track_err,max_track_err,solver_failures,killed,t_solve_mean" > "$OUT"
fi

while read -r entry; do
  case "$entry" in ''|'#'*) continue ;; esac
  set -- $entry
  label=$1; wp=$2; wn=$3; wt=$4; lat=$5; lon=$6; width=$7; ratio=$8; margin=$9

  # Restore before every case: overrides must never accumulate.
  cp "$SCRATCH/ptune2_ctrl_backup.yaml" "$CFG"
  cp "$SCRATCH/ptune2_plan_backup.yaml" "$PCFG"
  python3 - "$wp" "$wn" "$wt" "$lat" "$lon" "$width" "$ratio" "$CFG" <<'PY'
import io, re, sys
wp, wn, wt, lat, lon, width, ratio, path = sys.argv[1:9]
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
           "  progressmpc_max_slip_ratio: %s" % ratio, s)
if s == before:
    raise SystemExit("no config key matched - the sweep would have run the wrong settings")
io.open(path, "w", encoding="utf-8").write(s)
PY
  if [ $? -ne 0 ]; then echo "  CONFIG WRITE FAILED for $label"; continue; fi
  sed -i "s/^  smoothing_safety_margin: .*/  smoothing_safety_margin: $margin/" "$PCFG"

  echo "=== $label  prog=$wp n=$wn th=$wt lat=$lat long=$lon width=$width ratio=$ratio margin=$margin"
  ( cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash \
    && python3 src/control/test/supermpc_sweep.py --progress --repeats "$REPEATS" \
         --timeout 70 --max-cones 40 --max-lap-time 90 --max-error 8.0 \
         --out "$SCRATCH/pt2_${label}.csv" ) 2>&1 | grep -E "lap 1:|abandoned"

  python3 - "$label" "$SCRATCH/pt2_${label}.csv" "$OUT" <<'PY'
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
