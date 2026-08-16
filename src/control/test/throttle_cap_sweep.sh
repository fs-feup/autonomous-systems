#!/bin/bash
# Sweep the traction cap on commanded throttle.
#
# Mechanism, traced rather than inferred: the controller commands 0.61-0.80 throttle while the
# driven wheels sit at 100% slip from 3 m/s onward, long before the tracking error grows. The
# tyres can only accept about mu*m*g*rear_fraction, which is ~0.43 of peak motor torque. The MPC
# cannot predict the wheelspin (its wheel inertia is inflated 10x for QP conditioning, which
# testing showed cannot be undone at this discretisation), so it must be forbidden from asking.
#
# At cap 0.45 the correlation is stark: every run spending under 5% of the lap in wheelspin
# finished clean, both runs above 30% failed, and two runs had ZERO wheelspin - a first. So the
# quantity to minimise is the wheelspin FRACTION, not the peak, and this sweep looks for the cap
# at which it reaches zero without giving away more lap time than necessary.
#
# The cap is a bound on an existing state, so nothing here needs a rebuild.
WS=/home/ws
CFG=$WS/config/control/invictasim.yaml
OUT=$WS/src/control/test/mpcc_data/throttle_cap.csv
SCRATCH=/tmp/claude-1000/-home-ws/7f9690f5-09b3-4ebe-aeee-de085fcf1014/scratchpad
REPEATS=${REPEATS:-6}

cp "$CFG" "$SCRATCH/tcap_backup.yaml"
trap 'cp "$SCRATCH/tcap_backup.yaml" "$CFG"' EXIT

echo "case,repeat,time,cones_hit,avg_track_err,solver_failures,killed,slip_max,slip_frac_over_05" > "$OUT"

for cap in 0.30 0.35 0.40 0.50 0.55; do
  cp "$SCRATCH/tcap_backup.yaml" "$CFG"
  sed -i "s/^  progressmpc_max_throttle: .*/  progressmpc_max_throttle: $cap/" "$CFG"
  grep -q "progressmpc_max_throttle: $cap" "$CFG" || { echo "  CAP NOT SET for $cap"; continue; }

  echo "=== cap$cap"
  ( cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash \
    && python3 src/control/test/supermpc_sweep.py --progress --repeats "$REPEATS" \
         --timeout 70 --max-cones 40 --max-lap-time 90 --max-error 8.0 --max-slip-ratio 0.9 \
         --out "$SCRATCH/tcap_${cap}.csv" ) 2>&1 | grep -E "lap 1:|abandoned"

  # Confirm the cap was actually in force. A silently inactive cap already cost one full
  # experiment this session, so every case is verified from the controller's own log.
  active=$(grep -oE "throttle <= [0-9.]+" /tmp/supermpc_logs/ctrl_progress_default_r1.log 2>/dev/null | tail -1)
  echo "    [$active]"

  python3 - "cap$cap" "$SCRATCH/tcap_${cap}.csv" "$OUT" <<'PY'
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
print("  --- cap | clean | completed | best lap | mean wheelspin fraction ---")
for c, rs in g.items():
    clean = [r for r in rs if r["cones_hit"] == "0" and r["time"] not in ("", "nan", None)]
    done = [r for r in rs if r["time"] not in ("", "nan", None)]
    fr = [float(r["slip_frac_over_05"]) for r in rs
          if r["slip_frac_over_05"] not in ("", "nan", None)]
    print("    %-8s %d/%d   %d/%d   %8s   %s" % (
        c, len(clean), len(rs), len(done), len(rs),
        "%.2f" % min(float(r["time"]) for r in clean) if clean else "-",
        "%.3f" % (sum(fr) / len(fr)) if fr else "-"))
PY
done
echo "=== done ==="
