# pylint: skip-file
# mypy: ignore-errors
"""Parameter sweep for the SuperMPC contouring controller.

Runs one simulation per parameter set and records what the simulator reports: lap time, cones
hit and tracking error. Bad sets are abandoned as soon as they prove themselves bad rather than
burning the full timeout.

Usage:  python3 supermpc_sweep.py --out results.csv [--repeats 2]
"""
import argparse
import csv
import io
import os
import re
import signal
import subprocess
import time

import yaml
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from custom_interfaces.msg import WheelScalars
from custom_interfaces.msg import ControlStatistics, LapCurrent, LapSummary

WS = "/home/ws"
CONTROL_CFG = os.path.join(WS, "config/control/invictasim.yaml")
PLANNING_CFG = os.path.join(WS, "config/planning/invictasim.yaml")

# Stage cost order:
# [contour, heading, lag, ambition, excess, progress, vy, throttle, steer, thr_rate, str_rate]
BASE = [50.0, 10.0, 0.3, 2.0, 40.0, 0.0, 6.0, 0.05, 0.3, 2.0, 4.0]
CONTOUR, HEADING, LAG, AMBITION, EXCESS, PROGRESS = 0, 1, 2, 3, 4, 5


def weights(**kw):
    w = list(BASE)
    for name, value in kw.items():
        w[globals()[name.upper()]] = value
    return w


def build_cases_progress():
    """Smoke test for ProgressMPC at its default settings."""
    return [("progress_default", {"controller": "progressmpc"})]


def build_cases_ratio3():
    """Push contour higher still, with progress kept low.

    On the clean build, contour 80 tracks at 0.10-0.27 m while contour 300 with progress 3 gets
    to 0.063 m - better than mpczinho's 0.090 - so the tuned vector's contour was far too low for
    this solver. The stale YAML that predates all of this carried contour 600, which now looks
    deliberate rather than arbitrary. Every case so far still collects cones despite the good
    average, and max error sits near 1.0 m in all of them, so this also checks whether the
    excursion shrinks as contour rises or is independent of it.
    """
    def w(contour, progress, heading=10.0):
        return [contour, heading, 0.3, 0.0, 40.0, progress, 6.0, 0.05, 0.3, 2.0, 4.0]
    cases = []
    for contour in (300.0, 600.0, 1000.0):
        for progress in (1.0, 3.0):
            cases.append(("c%gp%g" % (contour, progress),
                          {"controller": "supermpc",
                           "supermpc_cost_weights": w(contour, progress)}))
    # Heading was tuned against the stale install too; 600/20 is the old YAML pairing.
    cases.append(("c600p3_head20", {"controller": "supermpc",
                                    "supermpc_cost_weights": w(600.0, 3.0, 20.0)}))
    cases.append(("Z_mpczinho", {"controller": "decoupled",
                                 "lateral_controller": "mpczinho"}))
    return cases


def build_cases_ratio2():
    """Re-tune the progress:contour ratio on the freshly built solver.

    The weight vector in use was tuned against an install that turned out to be stale, and on a
    clean build it tracks at ~0.40 m instead of ~0.08. The suspect is the progress term: its
    residual is (v_cap - progress_rate), so a heavy weight drives the virtual point along the
    path at the ceiling speed whatever the car is doing. Once PROGRESS outruns P_S_REF the
    reference slides along the stage TANGENT, which leaves a curved path - so the controller
    ends up chasing a point off the track exactly where it corners.

    Progress must stay > 0 or the QP is singular in that direction, so the sweep spans 1..35
    rather than including 0, and pairs the low end with a couple of contour levels.
    """
    def w(contour, progress):
        return [contour, 10.0, 0.3, 0.0, 40.0, progress, 6.0, 0.05, 0.3, 2.0, 4.0]
    cases = []
    for progress in (1.0, 3.0, 8.0, 20.0, 35.0):
        cases.append(("prog%g" % progress,
                      {"controller": "supermpc", "supermpc_cost_weights": w(80.0, progress)}))
    for contour in (150.0, 300.0):
        cases.append(("contour%g_prog3" % contour,
                      {"controller": "supermpc", "supermpc_cost_weights": w(contour, 3.0)}))
    return cases


def build_cases_diag():
    """mpczinho only, as a global control on the sim and the machine.

    SuperMPC regressed hard after a rebuild and neither candidate tyre setting brought it back,
    so the question is whether the regression is in the SuperMPC solver or in something shared.
    mpczinho does not use that solver at all and its lap time was very repeatable (25.09, sd
    0.026 over 60 runs), so if it still lands there the sim and machine are unchanged.
    """
    return [("diag_mpczinho", {"controller": "decoupled", "lateral_controller": "mpczinho"})]


def build_cases_horizon(label):
    """One case, named by the caller. The horizon itself is baked into the solver at codegen
    time, so horizon_sweep.sh sets it in YAML and rebuilds between invocations; all this has to
    do is run the winning control/planner config and record the result under the right label.
    """
    best = [80.0, 10.0, 0.3, 0.0, 40.0, 35.0, 6.0, 0.05, 0.3, 2.0, 4.0]
    return [(label, {"controller": "supermpc", "supermpc_cost_weights": best})]


def build_cases_planner3():
    """Confirmation run for a_lat 15, with enough repeats to put error bars on it.

    a_lat 15 came out 4/4 clean at 25.33 s mean against a 25.61 s baseline (sd 0.063), beating it
    on pace and reliability at once. The cliff either side is sharp - 14 is slow but safe, 16 is
    1/3, 18 is 0/3 - so 15.5 asks whether there is anything left before it. Margin stays at 0.6,
    now known to be a two-sided optimum (0.3/0.45 fatal, 0.75/0.9 slower).
    """
    best = [80.0, 10.0, 0.3, 0.0, 40.0, 35.0, 6.0, 0.05, 0.3, 2.0, 4.0]
    base = {"controller": "supermpc", "supermpc_cost_weights": best}
    cases = []
    for a_lat in (14.0, 15.0, 15.5):
        entry = dict(base)
        entry["planning.vp_lateral_acceleration"] = a_lat
        cases.append(("alat%g" % a_lat, entry))
    cases.append(("Z_mpczinho", {"controller": "decoupled",
                                 "lateral_controller": "mpczinho"}))
    return cases


def build_cases_planner2():
    """Follow-up to the first planner sweep, which settled three things.

    Baseline is tight: 7/9 clean at 25.61 s, sd 0.063 s, so a 0.1 s shift is measurable and the
    single 24.79 s run at a_lat 16 is a real 12-sigma gain, not scatter.

    Raising the speed ceiling did nothing (vmax 23/26/30 all 25.60-25.62), so pace is limited by
    cornering, not by top speed. Dropping the safety margin widens the corridor toward the cones
    and was uniformly fatal (0/3 clean at both 0.45 and 0.3). Both axes are closed.

    That leaves one live hypothesis: take the pace from a_lat 16 and buy the reliability back by
    moving the margin the other way, which no case has tried.
    """
    best = [80.0, 10.0, 0.3, 0.0, 40.0, 35.0, 6.0, 0.05, 0.3, 2.0, 4.0]
    base = {"controller": "supermpc", "supermpc_cost_weights": best}
    cases = []

    def case(label, **planning):
        entry = dict(base)
        entry.update({"planning." + k: v for k, v in planning.items()})
        cases.append((label, entry))

    # Bisect the cliff: 14 is 3/3 clean, 16 is 1/3.
    case("alat15", vp_lateral_acceleration=15.0)
    # A wider margin holds the line further from the cones, which is what a_lat 16 needs.
    for margin in (0.75, 0.9):
        case("alat16_m%g" % margin, vp_lateral_acceleration=16.0,
             smoothing_safety_margin=margin)
    case("alat15_m0.75", vp_lateral_acceleration=15.0, smoothing_safety_margin=0.75)
    # Is a bigger margin alone worth anything at the reliable a_lat?
    case("m0.75", smoothing_safety_margin=0.75)
    cases.append(("Z_mpczinho", {"controller": "decoupled",
                                 "lateral_controller": "mpczinho"}))
    return cases


def build_cases_planner():
    """Planner tuning, now that the controller tracks to ~0.07 m.

    Raising the speed ceiling on the controller side has failed in every campaign, so the pace
    limit is not the cap - it is the speed profile the planner asks for. With the line being
    held to a few centimetres the corridor can also be narrowed, which is what buys a shorter
    path rather than merely a faster one.
    """
    best = [80.0, 10.0, 0.3, 0.0, 40.0, 35.0, 6.0, 0.05, 0.3, 2.0, 4.0]
    base = {"controller": "supermpc", "supermpc_cost_weights": best}
    cases = []

    def case(label, **planning):
        entry = dict(base)
        entry.update({"planning." + k: v for k, v in planning.items()})
        cases.append((label, entry))

    for a_lat in (14.0, 16.0, 18.0):
        case("alat%g" % a_lat, vp_lateral_acceleration=a_lat)
    for margin in (0.3, 0.45, 0.6):
        case("margin%g" % margin, smoothing_safety_margin=margin)
    for vmax in (23.0, 26.0, 30.0):
        case("vmax%g" % vmax, vp_desired_velocity=vmax)
    # The combination the tight tracking is supposed to unlock.
    case("alat16_margin0.3", vp_lateral_acceleration=16.0, smoothing_safety_margin=0.3)
    case("alat16_margin0.3_v26", vp_lateral_acceleration=16.0,
         smoothing_safety_margin=0.3, vp_desired_velocity=26.0)
    cases.append(("Z_mpczinho", {"controller": "decoupled",
                                 "lateral_controller": "mpczinho"}))
    return cases


def build_cases_refine():
    """Refinement around the first configuration that was clean on every run.

    progress=20 was 5/5 where progress=10 was 2/5, so the reliability trend with progress has
    not turned over yet. This pushes it further and crosses it with the contour value that gave
    the lowest tracking error, plus a lower lag which produced the quickest single lap.
    """
    def w(contour=80.0, heading=10.0, lag=0.3, ambition=0.0, excess=40.0, progress=20.0,
          vy=6.0, throttle=0.05, steer=0.3, thr_rate=2.0, str_rate=4.0):
        return [contour, heading, lag, ambition, excess, progress, vy, throttle, steer,
                thr_rate, str_rate]
    cases = []
    for prog in (20.0, 35.0, 60.0):
        for contour in (80.0, 120.0):
            cases.append(("p%g_c%g" % (prog, contour),
                          {"controller": "supermpc",
                           "supermpc_cost_weights": w(progress=prog, contour=contour)}))
    # Lag interacts with progress: both act along the path, so the pair is crossed rather than
    # carrying the best lag value over from a different progress setting.
    for lag in (1.0, 2.0):
        cases.append(("p35_lag%g" % lag,
                      {"controller": "supermpc",
                       "supermpc_cost_weights": w(progress=35.0, lag=lag)}))
    # With the braking fixed, the speed ceiling can be revisited: it previously failed only
    # because the car was already unstable.
    for cap in (1.05, 1.15):
        cases.append(("p35_cap%g" % cap,
                      {"controller": "supermpc", "supermpc_cost_weights": w(progress=35.0),
                       "supermpc_speed_cap": cap}))
    cases.append(("Z_mpczinho", {"controller": "decoupled",
                                 "lateral_controller": "mpczinho"}))
    return cases


def build_cases_confirm():
    """Confirmation at higher repeat count.

    Raising progress from 1 to 10 took the clean rate from roughly 1-in-3 to 2-of-2 across six
    configurations, which is consistent with the braking mechanism the traces showed. Two runs
    each is still thin, so the survivors and the baseline are repeated enough to separate them.
    """
    def w(contour=80.0, heading=10.0, lag=0.3, ambition=0.0, excess=40.0, progress=10.0,
          vy=6.0, throttle=0.05, steer=0.3, thr_rate=2.0, str_rate=4.0):
        return [contour, heading, lag, ambition, excess, progress, vy, throttle, steer,
                thr_rate, str_rate]
    return [
        ("A_rates2_prog10", {"controller": "supermpc", "supermpc_cost_weights": w()}),
        ("B_rates4_prog10", {"controller": "supermpc",
                             "supermpc_cost_weights": w(thr_rate=4.0, str_rate=8.0)}),
        ("C_lag2_prog10", {"controller": "supermpc",
                           "supermpc_cost_weights": w(lag=2.0)}),
        ("D_prog20", {"controller": "supermpc", "supermpc_cost_weights": w(progress=20.0)}),
        ("E_contour120_prog10", {"controller": "supermpc",
                                 "supermpc_cost_weights": w(contour=120.0)}),
        ("Z_mpczinho", {"controller": "decoupled", "lateral_controller": "mpczinho"}),
    ]


def build_cases_ratio():
    """The progress:contour ratio, plus the rate sweet spot.

    The failure traces showed the controller commanding negative throttle while far below
    target speed and braking harder as error grew, ending stopped. At contour 80 vs progress 1
    the cheapest way to stop contour error accumulating is to stop moving, so these raise the
    value of forward motion until it outweighs the error it accrues.

    Rates are swept finely in the low range as well: heavier damping visibly steadied the car
    but too much makes it laggy, so the useful setting is an interior optimum, not a limit.
    """
    def w(contour=80.0, heading=10.0, lag=0.3, ambition=0.0, excess=40.0, progress=1.0,
          vy=6.0, throttle=0.05, steer=0.3, thr_rate=6.0, str_rate=12.0):
        return [contour, heading, lag, ambition, excess, progress, vy, throttle, steer,
                thr_rate, str_rate]

    cases = [("ref_mpczinho", {"controller": "decoupled", "lateral_controller": "mpczinho"})]

    def sm(label, wl, **extra):
        e = {"controller": "supermpc", "supermpc_cost_weights": wl}
        e.update(extra)
        cases.append((label, e))

    for prog in (2.0, 5.0, 10.0, 20.0, 40.0):
        sm("prog%g" % prog, w(progress=prog))
    for contour in (20.0, 40.0):
        sm("contour%g_prog10" % contour, w(contour=contour, progress=10.0))
    for lag in (2.0, 6.0):
        sm("lag%g_prog10" % lag, w(lag=lag, progress=10.0))
    # Rate sweet spot, at a progress high enough that the car should not brake itself down.
    for thr, strr in ((2.0, 4.0), (4.0, 8.0), (6.0, 12.0), (10.0, 20.0), (15.0, 30.0)):
        sm("rates%g_prog10" % thr, w(progress=10.0, thr_rate=thr, str_rate=strr))
    return cases


def build_cases_stability():
    """Stability-first sweep.

    Two ideas drive it. First, the observation that heavier command-rate weights made the car
    visibly steadier: rate weights are also what damps the solver's own chatter, so they are
    swept hard and in combination rather than one at a time. Second, that departures may be
    numerical rather than physical - an ill-conditioned cost (huge ratios between weights) can
    make the QP produce nonsense long before the car is anywhere near the tyre limit, so the
    extremes here are deliberately paired with a well-conditioned base.
    """
    def w(contour=80.0, heading=10.0, lag=0.3, ambition=0.0, excess=40.0, progress=1.0,
          vy=6.0, throttle=0.05, steer=0.3, thr_rate=2.0, str_rate=4.0):
        return [contour, heading, lag, ambition, excess, progress, vy, throttle, steer,
                thr_rate, str_rate]

    cases = [("ref_mpczinho", {"controller": "decoupled", "lateral_controller": "mpczinho"})]

    def sm(label, weights_list, **extra):
        entry = {"controller": "supermpc", "supermpc_cost_weights": weights_list}
        entry.update(extra)
        cases.append((label, entry))

    # Rate weights swept together, since damping one actuator while leaving the other free
    # just moves the chatter rather than removing it.
    for thr, strr in ((2.0, 4.0), (6.0, 12.0), (15.0, 30.0), (40.0, 80.0), (100.0, 200.0)):
        sm("rates_%g_%g" % (thr, strr), w(thr_rate=thr, str_rate=strr))

    # Command magnitudes alongside the rates: together these decide how hard the solver is
    # allowed to move the actuators at all.
    for mag in (0.3, 1.0, 3.0):
        sm("mag%g_rates15" % mag, w(throttle=mag / 6.0, steer=mag, thr_rate=15.0, str_rate=30.0))

    # Conditioning: the same relative shape, scaled. If departures are numerical rather than
    # physical, the badly scaled version fails and the compressed one does not.
    sm("cond_compressed", w(contour=8.0, heading=1.0, excess=4.0, progress=0.1,
                            vy=0.6, thr_rate=1.5, str_rate=3.0))
    sm("cond_stretched", w(contour=800.0, heading=100.0, excess=400.0, progress=10.0,
                           vy=60.0, thr_rate=20.0, str_rate=40.0))

    # Best-known shape combined with the heavier damping.
    for contour in (50.0, 80.0, 150.0):
        sm("contour%g_rates15" % contour, w(contour=contour, thr_rate=15.0, str_rate=30.0))
    for progress in (0.5, 1.0, 2.0):
        sm("prog%g_rates15" % progress, w(progress=progress, thr_rate=15.0, str_rate=30.0))
    # The progress:contour ratio, which the failure traces point at directly. At 80:1 the
    # cheapest way to stop contour error accumulating is to stop moving, so the solver brakes
    # into a standstill once the car is off the line. These raise progress until forward
    # motion is worth more than the error it accrues.
    for progress in (5.0, 10.0, 20.0, 40.0):
        sm("ratio_prog%g" % progress, w(progress=progress, thr_rate=15.0, str_rate=30.0))
    for contour in (20.0, 40.0):
        sm("ratio_contour%g_prog10" % contour,
           w(contour=contour, progress=10.0, thr_rate=15.0, str_rate=30.0))
    # Lag also pulls the car forward when it falls behind, so it is raised alongside progress.
    for lag in (2.0, 6.0):
        sm("ratio_lag%g_prog10" % lag,
           w(lag=lag, progress=10.0, thr_rate=15.0, str_rate=30.0))

    for slip in (0.20, 0.30, 0.40):
        sm("slip%g_rates15" % slip, w(thr_rate=15.0, str_rate=30.0),
           supermpc_max_rear_slip=slip)
    return cases


def build_cases_deadline():
    """Isolates the control period from the tuning.

    The weight sweep showed lap time pinned at 25-26 s across a 6.7x swing in the dominant
    weight, with failures hitting the baseline controller too. That is the signature of a
    timing fault rather than a cost fault: at 25 ms command interval the solver was using
    ~23 ms, so an overrun feeds the plant a command computed from a stale state. This compares
    the same controller at two command intervals, against the baseline at both.
    """
    best = weights(contour=80.0, heading=10.0, lag=0.3, ambition=0.0, excess=40.0, progress=1.0)
    cases = []
    for interval in (25, 40):
        for i in range(3):
            cases.append(("mpczinho_%dms_%d" % (interval, i + 1),
                          {"controller": "decoupled", "lateral_controller": "mpczinho",
                           "command_time_interval": interval}))
        for i in range(3):
            cases.append(("supermpc_%dms_%d" % (interval, i + 1),
                          {"controller": "supermpc", "supermpc_cost_weights": best,
                           "command_time_interval": interval}))
    return cases


def build_cases_full():
    """Broad one-factor-at-a-time campaign over every cost axis, plus a matched baseline.

    One factor at a time rather than a factorial: eleven weights cannot be crossed at any
    sensible run count, and only the RATIOS between them matter, so scaling several at once
    mostly reproduces points already covered. Each family moves one ratio and holds the rest at
    the best configuration found so far.
    """
    base = dict(contour=80.0, heading=10.0, lag=0.3, ambition=0.0, excess=40.0, progress=1.0)
    cases = []

    # Matched baseline: the controller supermpc has to beat, measured in the same session and
    # under the same machine load as everything else.
    for i in range(3):
        cases.append(("baseline_mpczinho_%d" % (i + 1),
                      {"controller": "decoupled", "lateral_controller": "mpczinho"}))

    def sm(label, **kw):
        merged = dict(base)
        merged.update({k: v for k, v in kw.items() if k in base})
        extra = {k: v for k, v in kw.items() if k not in base}
        entry = {"controller": "supermpc", "supermpc_cost_weights": weights(**merged)}
        entry.update(extra)
        cases.append((label, entry))

    for v in (30.0, 50.0, 80.0, 120.0, 200.0):
        sm("contour%g" % v, contour=v)
    for v in (0.5, 0.75, 1.0, 1.5, 2.0):
        sm("progress%g" % v, progress=v)
    for v in (0.1, 0.3, 0.6, 1.0):
        sm("lag%g" % v, lag=v)
    for v in (5.0, 10.0, 20.0, 40.0):
        sm("heading%g" % v, heading=v)
    for v in (10.0, 40.0, 100.0):
        sm("excess%g" % v, excess=v)

    # Regularisation: sideslip, the two command magnitudes and the two command rates. These are
    # what stop the solver from chattering the actuators, and they were never swept.
    for v in (1.0, 3.0, 6.0, 12.0):
        w = weights(**base); w[6] = v
        cases.append(("vy%g" % v, {"controller": "supermpc", "supermpc_cost_weights": w}))
    for v in (0.01, 0.05, 0.2):
        w = weights(**base); w[7] = v
        cases.append(("throttle%g" % v, {"controller": "supermpc", "supermpc_cost_weights": w}))
    for v in (0.1, 0.3, 1.0):
        w = weights(**base); w[8] = v
        cases.append(("steer%g" % v, {"controller": "supermpc", "supermpc_cost_weights": w}))
    for v in (0.5, 2.0, 8.0):
        w = weights(**base); w[9] = v
        cases.append(("thrrate%g" % v, {"controller": "supermpc", "supermpc_cost_weights": w}))
    for v in (1.0, 4.0, 10.0):
        w = weights(**base); w[10] = v
        cases.append(("strrate%g" % v, {"controller": "supermpc", "supermpc_cost_weights": w}))

    # Speed ceiling and the rear-slip envelope: the two hard limits on how much the controller
    # is allowed to ask of the tyres.
    for v in (1.0, 1.02, 1.10):
        sm("cap%g" % v, supermpc_speed_cap=v)
    for v in (0.10, 0.15, 0.25, 0.40):
        sm("slip%g" % v, supermpc_max_rear_slip=v)

    # Terminal weights decide how much the end of the horizon is trusted.
    for terminal in ([10.0, 5.0, 0.3, 0.25], [30.0, 10.0, 0.3, 0.25], [100.0, 30.0, 1.0, 0.25]):
        sm("terminal%g" % terminal[0], supermpc_terminal_cost_weights=terminal)

    return cases


def build_cases_stage2():
    """Confirmation sweep, re-based on the setting that actually worked.

    The first sweep built its contour and lag families on progress=8, which turned out to be a
    bad operating point, so those runs measured the base rather than the axis under test. Both
    families are rebuilt here on progress=1, and every case is repeated because single runs on
    this stack cannot separate a real effect from run-to-run scatter.
    """
    cases = [("base_prog1", {"supermpc_cost_weights": weights(ambition=0.0, progress=1.0)}),
             ("prog0.5", {"supermpc_cost_weights": weights(ambition=0.0, progress=0.5)}),
             ("prog2", {"supermpc_cost_weights": weights(ambition=0.0, progress=2.0)})]
    for contour in (80.0, 150.0):
        cases.append(("prog1_contour%g" % contour,
                      {"supermpc_cost_weights": weights(contour=contour, ambition=0.0,
                                                        progress=1.0)}))
    for lag in (0.1, 1.0):
        cases.append(("prog1_lag%g" % lag,
                      {"supermpc_cost_weights": weights(lag=lag, ambition=0.0, progress=1.0)}))
    for cap in (1.15, 1.30):
        cases.append(("prog1_cap%g" % cap,
                      {"supermpc_cost_weights": weights(ambition=0.0, progress=1.0),
                       "supermpc_speed_cap": cap}))
    # A target-mode control, so the two readings of the planner speed are compared fairly.
    cases.append(("target_amb2_prog0.2",
                  {"supermpc_cost_weights": weights(ambition=2.0, progress=0.2)}))
    return cases


def build_cases():
    """Two families, matching the two readings of the planner's speed.

    'target'  - the planner speed is a rough target: the ambition term carries the weight.
    'ceiling' - the planner speed is only an upper bound: the progress term carries it, and
                the controller picks its own pace subject to the tyre model and the cap.

    The progress weight is never zero: it is the only cost term acting on the progress rate,
    and without it that input is unpenalised and the QP is singular in that direction.
    """
    cases = []

    # Family A: planner as a rough target.
    for ambition in (1.0, 2.0, 4.0):
        cases.append(("target_amb%g" % ambition,
                      {"supermpc_cost_weights": weights(ambition=ambition, progress=0.2)}))

    # Family B: planner as a ceiling only; progress drives the pace.
    for progress in (1.0, 3.0, 8.0, 15.0):
        cases.append(("ceiling_prog%g" % progress,
                      {"supermpc_cost_weights": weights(ambition=0.0, progress=progress)}))

    # Family B with a raised ceiling, so the cap stops being the binding constraint.
    for cap in (1.15, 1.30):
        cases.append(("ceiling_prog8_cap%g" % cap,
                      {"supermpc_cost_weights": weights(ambition=0.0, progress=8.0),
                       "supermpc_speed_cap": cap}))

    # How tightly the line is held, at the best-guess pace setting.
    for contour in (30.0, 80.0, 150.0):
        cases.append(("contour%g" % contour,
                      {"supermpc_cost_weights": weights(contour=contour, ambition=0.0,
                                                        progress=8.0)}))

    # Lag decides how free the timing is. Too small and the reference stops anchoring the car.
    for lag in (0.1, 1.0):
        cases.append(("lag%g" % lag,
                      {"supermpc_cost_weights": weights(lag=lag, ambition=0.0, progress=8.0)}))

    return cases


class Collector(Node):
    # index 0 of /acados/execution_times is t_tot in ms

    def __init__(self, context):
        super().__init__("supermpc_sweep", context=context)
        self.laps = {}
        self.solve_times = []
        self.create_subscription(Float64MultiArray, "/acados/execution_times",
                                 self._solve_time, 10)
        self.live_cones = 0
        self.live_time = 0.0
        # Live path error, so a car that has wandered off the track is abandoned immediately
        # instead of being left to drift around the infield until the lap-time bound expires.
        self.live_error = 0.0
        self.error_since = None
        # Driven-wheel slip ratio. Past ~0.5 the tyre is well beyond its force peak and the run
        # is just wheelspin - it cannot produce a meaningful lap, so there is nothing to learn by
        # letting it play out.
        self.live_slip = 0.0
        self.slip_since = None
        self.live_speed = 0.0
        self.slip_samples = []
        self.create_subscription(WheelScalars, "/invictasim/vehicle_model/tire/slip_ratio",
                                 self._slip, 10)
        self.create_subscription(ControlStatistics, "/invictasim/statistics/control_statistics",
                                 self._stats, 10)
        self.create_subscription(LapSummary, "/invictasim/statistics/lap_summary",
                                 self._summary, 10)
        self.create_subscription(LapCurrent, "/invictasim/statistics/lap_current",
                                 self._current, 10)

    def _slip(self, msg):
        # Only the driven wheels: the fronts are dragged and their ratio says nothing about the
        # optimizer over-driving the powertrain.
        self.live_slip = max(abs(msg.rl), abs(msg.rr))
        # Sampled only where the quantity means something. Recording the distribution is what
        # makes the kill threshold checkable instead of assumed: a threshold of 0.5 turned out to
        # abandon configurations that had previously produced clean 25.5 s laps.
        if self.live_speed > kSlipKillMinSpeed:
            self.slip_samples.append(self.live_slip)

    def _solve_time(self, msg):
        if msg.data:
            self.solve_times.append(msg.data[0])

    def _stats(self, msg):
        self.live_error = abs(msg.tracking_error)
        self.live_speed = msg.current_velocity

    def _current(self, msg):
        self.live_cones = msg.current_lap_cones_hit
        self.live_time = msg.current_lap_time

    def _summary(self, msg):
        for row in msg.rows:
            if row.time > 0.0:
                self.laps[row.lap_number] = {
                    "lap": row.lap_number, "time": row.time, "cones_hit": row.cones_hit,
                    "avg_velocity": row.avg_velocity, "max_velocity": row.max_velocity,
                    "avg_track_err": row.avg_tracking_error_distance,
                    "max_track_err": row.max_tracking_error_distance,
                }


def render(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return '"%s"' % value
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(str(v) for v in value) + "]"
    return str(value)


def _set_key(text, key, value, path):
    pattern = re.compile(r"^(\s+)(%s):([^#\n]*)(#.*)?$" % re.escape(key), re.M)
    text, n = pattern.subn(
        lambda m: "%s%s: %s%s" % (m.group(1), m.group(2), render(value),
                                  (" " + m.group(4)) if m.group(4) else ""), text)
    if n == 0:
        raise SystemExit("key not found in %s: %s" % (path, key))
    return text


BASELINE = {}


def capture_baseline():
    """Snapshots both configs so every case starts from the same file.

    Without this each case is applied on top of whatever the previous case left behind, so a
    sweep that varies one key per case silently accumulates every key it has ever set. That
    turns a one-factor-at-a-time design into an uncontrolled walk through the parameter space.
    """
    BASELINE[CONTROL_CFG] = io.open(CONTROL_CFG, encoding="utf-8").read()
    BASELINE[PLANNING_CFG] = io.open(PLANNING_CFG, encoding="utf-8").read()


def apply_overrides(overrides):
    """Line edits, not a YAML round trip: dumping the document back out strips every comment.

    Keys prefixed 'planning.' go to the planner config, everything else to the controller, so a
    single case can move both sides at once. Always applied to the pristine baseline, never to
    the previous case's output.
    """
    files = {CONTROL_CFG: ("control", BASELINE[CONTROL_CFG]),
             PLANNING_CFG: ("planning", BASELINE[PLANNING_CFG])}
    for path, (root, text) in files.items():
        if "{" in text.split("\n", 1)[0]:
            raise SystemExit("refusing to edit flow-style YAML: " + path)

    counts = {p: len(yaml.safe_load(t)[r]) for p, (r, t) in files.items()}
    for key, value in overrides.items():
        if key.startswith("planning."):
            root, text = files[PLANNING_CFG]
            files[PLANNING_CFG] = (root, _set_key(text, key[len("planning."):], value,
                                                  PLANNING_CFG))
        else:
            root, text = files[CONTROL_CFG]
            files[CONTROL_CFG] = (root, _set_key(text, key, value, CONTROL_CFG))

    for path, (root, text) in files.items():
        if len(yaml.safe_load(text)[root]) != counts[path]:
            raise SystemExit("edit changed the key count in " + path)
        io.open(path, "w", encoding="utf-8").write(text)


def kill_all():
    for pattern in ("lib/invictasim/invictasim", "ros2 launch invictasim",
                    "node_control", "lib/planning/planning"):
        subprocess.run(["pkill", "-9", "-f", pattern],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def spawn(cmd, log_path, env=None):
    log = open(log_path, "w")
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, preexec_fn=os.setsid,
                            env=env or os.environ.copy()), log


# Below this speed the slip ratio is not a meaningful quantity, so it is not judged.
kSlipKillMinSpeed = 3.0


def run_case(tag, log_dir, timeout_s, max_cones, max_lap_time, max_error, max_slip_ratio):
    kill_all()

    # Fresh context per case. Subscriptions outlive a simulator that is killed and relaunched
    # every case, and the shared context was observed to go invalid part-way through a sweep,
    # after which every spin raised and the whole run was lost.
    # An explicit Context per case, not the global one: the global context cannot be re-inited
    # after shutdown, and reusing a single long-lived context across a whole sweep was observed
    # to go invalid part-way through, after which every spin raised and the run was lost.
    context = rclpy.context.Context()
    rclpy.init(context=context)
    collector = Collector(context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(collector)
    collector.live_error = 0.0
    collector.error_since = None
    collector.live_slip = 0.0
    collector.slip_since = None
    collector.live_speed = 0.0
    collector.slip_samples = []

    env = os.environ.copy()
    # The launch file asks for wayland; forcing x11 here sends the simulator's rendering through
    # XWayland, which burns CPU the control node then has to compete with and inflates every
    # solve-time reading. "dummy" disables rendering outright, which is what a sweep wants.
    env["SDL_VIDEODRIVER"] = os.environ.get("SWEEP_SDL", "dummy")
    sim, sim_log = spawn(["ros2", "launch", "invictasim", "invictasim.launch.py"],
                         os.path.join(log_dir, "sim.log"), env)
    time.sleep(14.0)
    plan, plan_log = spawn(["ros2", "run", "planning", "planning"],
                           os.path.join(log_dir, "plan_%s.log" % tag))
    time.sleep(3.0)
    ctrl, ctrl_log = spawn(["ros2", "run", "control", "node_control"],
                           os.path.join(log_dir, "ctrl_%s.log" % tag))

    deadline = time.time() + timeout_s
    killed = None
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.2)
        if collector.live_cones > max_cones:
            killed = "cones>%d" % max_cones
            break
        # Off the track. Held for a moment before acting so a brief excursion on a kerb does
        # not kill an otherwise good run, but a car that has genuinely left the circuit is
        # abandoned in a second or two rather than after a full timeout.
        # Slip ratio is (r*omega - vx) / vx, so at a standing start it saturates to 1.0 for any
        # wheel rotation at all - it is undefined there rather than bad. Gating on speed is what
        # makes this measure wheelspin instead of measuring "the car launched": without the gate
        # it fired on every run regardless of the setting under test, and wrongly implicated the
        # wheel inertia.
        if collector.live_speed > kSlipKillMinSpeed and collector.live_slip > max_slip_ratio:
            if collector.slip_since is None:
                collector.slip_since = time.time()
            elif time.time() - collector.slip_since > 1.5:
                killed = "slip_ratio(%.2f)" % collector.live_slip
                break
        else:
            collector.slip_since = None

        if collector.live_error > max_error:
            if collector.error_since is None:
                collector.error_since = time.time()
            elif time.time() - collector.error_since > 1.5:
                killed = "off_track(err=%.1fm)" % collector.live_error
                break
        else:
            collector.error_since = None
        if collector.live_time > max_lap_time:
            killed = "lap>%.0fs" % max_lap_time
            break
        if [k for k in collector.laps if k > 0]:
            break

    for proc in (ctrl, plan, sim):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
    for log in (ctrl_log, plan_log, sim_log):
        log.close()
    kill_all()

    laps_snapshot = dict(collector.laps)
    slip_snapshot = list(collector.slip_samples)
    solve_snapshot = list(collector.solve_times)
    try:
        executor.shutdown()
        collector.destroy_node()
        rclpy.shutdown(context=context)
    except Exception:
        pass

    solver_failures = sum(
        1 for line in io.open(os.path.join(log_dir, "ctrl_%s.log" % tag),
                              encoding="utf-8", errors="ignore")
        if "MINSTEP" in line)

    laps = [laps_snapshot[k] for k in sorted(laps_snapshot) if k > 0]
    if not laps and killed is None:
        killed = "no_lap"
    t_mean = sum(solve_snapshot) / len(solve_snapshot) if solve_snapshot else float("nan")
    t_max = max(solve_snapshot) if solve_snapshot else float("nan")
    slip_max = max(slip_snapshot) if slip_snapshot else float("nan")
    slip_frac = (sum(1 for v in slip_snapshot if v > 0.5) / len(slip_snapshot)
                 if slip_snapshot else float("nan"))
    return laps, killed, solver_failures, t_mean, t_max, slip_max, slip_frac


INSTALLED_BINARY = "/home/ws/install/control/lib/control/node_control"
CONTROLLER_SOURCES = "/home/ws/src/control"


def verify_install_fresh(allow_stale):
    """Refuse to measure a binary that predates the sources it is supposed to contain.

    Twice now a full dataset has been collected against a stale install - once the numbers were
    attributed to a code change that was never compiled in, and reconciling that cost more time
    than every sweep in this file put together. `colcon build` exiting 0 is NOT evidence the
    install updated: the build can succeed while the install step is skipped, and nothing in the
    run output says so. Comparing mtimes is the only cheap check that actually catches it, so it
    is a hard failure rather than a warning - a warning scrolls past in an unattended campaign.
    """
    if not os.path.exists(INSTALLED_BINARY):
        raise SystemExit("no installed binary at %s - build first" % INSTALLED_BINARY)
    binary_mtime = os.path.getmtime(INSTALLED_BINARY)
    newer = []
    for root, _, files in os.walk(CONTROLLER_SOURCES):
        # Generated acados C is a build product, and test/ does not go into the binary.
        if "c_generated_code" in root or "/test" in root:
            continue
        for name in files:
            if not name.endswith((".cpp", ".hpp", ".py")):
                continue
            path = os.path.join(root, name)
            if os.path.getmtime(path) > binary_mtime:
                newer.append(path)
    if not newer:
        return
    message = ["STALE INSTALL: %d source file(s) are newer than the installed binary"
               % len(newer)]
    for path in sorted(newer)[:5]:
        message.append("  %s" % path)
    message.append("  binary: %s" % time.strftime("%m-%d %H:%M",
                                                  time.localtime(binary_mtime)))
    if allow_stale:
        message.append("  --allow-stale given, continuing anyway")
        print("\n".join(message), flush=True)
        return
    message.append("Rebuild, or pass --allow-stale if this is deliberate.")
    raise SystemExit("\n".join(message))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/tmp/supermpc_sweep.csv")
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=75.0)
    parser.add_argument("--max-cones", type=int, default=12)
    parser.add_argument("--max-lap-time", type=float, default=50.0)
    parser.add_argument("--log-dir", default="/tmp/supermpc_logs")
    parser.add_argument("--stage2", action="store_true")
    parser.add_argument("--full", action="store_true")
    parser.add_argument("--deadline", action="store_true")
    parser.add_argument("--stability", action="store_true")
    parser.add_argument("--ratio", action="store_true")
    parser.add_argument("--confirm", action="store_true")
    parser.add_argument("--refine", action="store_true")
    parser.add_argument("--planner", action="store_true")
    parser.add_argument("--planner2", action="store_true")
    parser.add_argument("--planner3", action="store_true")
    parser.add_argument("--horizon", action="store_true")
    parser.add_argument("--diag", action="store_true")
    parser.add_argument("--ratio2", action="store_true")
    parser.add_argument("--ratio3", action="store_true")
    parser.add_argument("--progress", action="store_true")
    parser.add_argument("--label", default="case")
    parser.add_argument("--max-slip-ratio", type=float, default=0.9,
                        help="abandon once a driven wheel exceeds this slip ratio for >1.5 s; "
                             "0.5 proved too strict - configurations that produced clean laps "
                             "cross it briefly, so slip is now recorded per run as well")
    parser.add_argument("--max-error", type=float, default=3.0,
                        help="abandon once the car is this far off the path for >1.5 s")
    parser.add_argument("--allow-stale", action="store_true",
                        help="run even if the installed binary is older than the controller "
                             "sources (use only when deliberately re-measuring an old build)")
    args = parser.parse_args()
    os.makedirs(args.log_dir, exist_ok=True)
    verify_install_fresh(args.allow_stale)

    original = io.open(CONTROL_CFG, encoding="utf-8").read()
    original_planning = io.open(PLANNING_CFG, encoding="utf-8").read()
    capture_baseline()
    rows = []
    fieldnames = ["case", "repeat", "lap", "time", "cones_hit", "avg_velocity", "max_velocity",
                  "avg_track_err", "max_track_err", "solver_failures", "killed", "t_solve_mean", "t_solve_max",
                  "slip_max", "slip_frac_over_05"]
    csv_handle = open(args.out, "w", newline="")
    csv_writer = csv.DictWriter(csv_handle, fieldnames=fieldnames, extrasaction="ignore")
    csv_writer.writeheader()
    csv_handle.flush()
    try:
        cases = (build_cases_progress() if args.progress else
                 build_cases_ratio3() if args.ratio3 else
                 build_cases_ratio2() if args.ratio2 else
                 build_cases_diag() if args.diag else
                 build_cases_horizon(args.label) if args.horizon else
                 build_cases_planner3() if args.planner3 else
                 build_cases_planner2() if args.planner2 else
                 build_cases_planner() if args.planner else
                 build_cases_refine() if args.refine else
                 build_cases_confirm() if args.confirm else
                 build_cases_ratio() if args.ratio else
                 build_cases_stability() if args.stability else
                 build_cases_deadline() if args.deadline else
                 build_cases_full() if args.full else
                 build_cases_stage2() if args.stage2 else build_cases())
        for index, (label, overrides) in enumerate(cases):
            for repeat in range(args.repeats):
                tag = label if args.repeats == 1 else "%s_r%d" % (label, repeat + 1)
                print("[%d/%d] %s" % (index + 1, len(cases), tag), flush=True)
                apply_overrides(overrides)
                (laps, killed, failures, t_mean, t_max,
                 slip_max, slip_frac) = run_case(tag, args.log_dir, args.timeout,
                                                  args.max_cones, args.max_lap_time,
                                                  args.max_error, args.max_slip_ratio)
                if not laps:
                    print("    abandoned: %s (solver failures %d)" % (killed, failures),
                          flush=True)
                    rows.append({"case": label, "repeat": repeat + 1, "time": float("nan"),
                                 "cones_hit": -1, "avg_track_err": float("nan"),
                                 "avg_velocity": float("nan"), "max_velocity": float("nan"),
                                 "solver_failures": failures, "killed": killed,
                                 "t_solve_mean": t_mean, "t_solve_max": t_max,
                                 "slip_max": slip_max, "slip_frac_over_05": slip_frac})
                    csv_writer.writerow(rows[-1]); csv_handle.flush()
                    continue
                for lap in laps:
                    print("    lap %d: %.2fs cones=%d err=%.3f vmax=%.1f failures=%d" %
                          (lap["lap"], lap["time"], lap["cones_hit"], lap["avg_track_err"],
                           lap["max_velocity"], failures), flush=True)
                    rows.append({"case": label, "repeat": repeat + 1, **lap,
                                 "solver_failures": failures, "killed": killed or "",
                                 "t_solve_mean": t_mean, "t_solve_max": t_max,
                                 "slip_max": slip_max, "slip_frac_over_05": slip_frac})
                    csv_writer.writerow(rows[-1]); csv_handle.flush()
    finally:
        io.open(CONTROL_CFG, "w", encoding="utf-8").write(original)
        io.open(PLANNING_CFG, "w", encoding="utf-8").write(original_planning)
        csv_handle.close()
        kill_all()

    # Rows were already streamed to disk as they completed; nothing to rewrite here.
    print("Wrote %s (%d rows)" % (args.out, len(rows)))


if __name__ == "__main__":
    main()
