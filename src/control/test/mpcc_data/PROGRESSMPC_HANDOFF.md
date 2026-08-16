# ProgressMPC — state at pause (2026-08-13)

New controller, built from scratch this session. Everything below is measured.

## What it is

A corridor-constrained, progress-maximising MPC. The path is **not** a reference: it supplies a
corridor the car must stay inside and the curvature of that corridor, and nothing else. The car
is asked only to cover arc length. Same vehicle model as SuperMPC/Bombated (Pacejka, powertrain,
differential, wheel dynamics) — copied byte-identical.

**Files added:**

| file | role |
|---|---|
| `include/solver/progressmpc_acados/progressmpc_acados.py` | solver codegen |
| `include/solver/progressmpc_acados/progressmpc_acados.hpp` / `src/solver/progressmpc_acados.cpp` | C++ solver |
| `include/controller/progressmpc.hpp` / `src/controller/progressmpc.cpp` | controller |
| `test/progress_sweep.sh`, `progress_sweep2.sh`, `progress_cases*.txt` | tuning campaigns |
| `test/inertia_ab.sh` | wheel-inertia experiment (complete; conclusion is negative) |
| `test/long_grip_sweep.sh` | longitudinal grip budget sweep |
| `test/solve_profile.py` | solve-time profiler by phase |

Also wired into `controller/map.hpp`, `solver/map.hpp`, `config/parameters.{hpp,cpp}`,
`CMakeLists.txt`, `config/control/invictasim.yaml`.

**Formulation.** States are curvilinear: `(S, N, THETA_E)` replace global `(x, y, yaw)`, so
NX 16, NU 2, NP 5, N=30 over 2.0 s. Cost is `(30 - s_dot)` plus small regularisers. Constraints
(`NH=6`, all soft): rear slip angle x2, lateral grip budget, corridor, rear slip ratio x2. The
longitudinal budget is a soft STATE bound on `AX` (`NBX=4`), not an h-row - see session 2.
Terminal (`NHN=2`): corridor and `vx <= v_max`, where `v_max` is the planner's speed at the
horizon end — the **only** place the planner's speed is used.

## Results

Benchmark on the same binary: **mpczinho 24.68 s / 0 cones / 0.090 m**, extremely repeatable.

Best ProgressMPC so far: **25.51 / 25.56 s, 0 cones, err 0.175, zero solver failures**
(lateral grip 0.70). Reliability is the open problem, not pace.

**Lateral grip utilisation is by far the dominant axis:**

| lateral λ | clean | lap |
|---|---|---|
| 1.00 | 0/3 | — |
| 0.85 | ~1/18 | 25.30 |
| 0.70 | 2/3 + 1/3 | 25.51 / 25.56 / 26.01 |
| 0.65 | 1/3 | 26.01 |
| 0.60 | 2/3 | 26.61 / 27.36 |

Pace is monotonic in λ. Reliability is noisy at n=3 — the between-repeat spread exceeds the
between-λ difference, so **do not** read the 0.60/0.65/0.70 ordering off three runs each. Re-run
the survivors at 6+ repeats.

Everything else (slip ratio, lateral weight, progress weight, corridor width, planner margin)
moved the result far less than λ.

## SESSION 2 (2026-08-14) — the inertia question is CLOSED, negatively

**True wheel inertia does not work and cannot be made to work at this discretisation.** Four
variants, all fail (`inertia_ab.sh`, data in `inertia_ab.csv`):

| inertia | IRK sub-steps | result |
|---|---|---|
| 0.2/0.4 (true) | 2 | 3/3 slip-killed |
| 0.2/0.4 (true) | 4 | 27-44 solver failures, one 47.8 s / 7-cone lap |
| 0.2/0.4 (true) | 3, N=20 | 3/3 slip-killed |
| 0.6/0.8 (middle) | 2 | 3/3 slip-killed |

The torque path was checked first and is identical to the sim (220 Nm, 124 kW, gear 3.67,
efficiency 0.97), so the overshoot is not a torque-map mismatch. The reason true inertia fails is
the one the original comment gave: the wheel-spin mode is sub-millisecond against 33 ms
sub-steps. IRK is implicit so it stays stable, but it SMOOTHS a mode that fast rather than
resolving it - the predictions become wrong differently, not right. Resolving it needs ~1 ms
sub-steps, roughly 67x the linearisation cost, when linearisation is already 13 ms of a 23 ms
solve. **Inflated inertia is a necessary approximation. Do not re-test this.**

The real remaining option for getting the wheel dynamics honest is structural: drop the wheel
speeds as dynamic states and solve slip quasi-statically, which removes the stiff mode instead of
approximating around it. Untried.

**The longitudinal grip budget must NOT be a nonlinear constraint row.** Added as a 7th soft
h-row it took the solver from zero failures to 24-49 per run. It is now a linear bound on the
filtered acceleration state `AX` (NBX 3->4), which is nearly free, and it is ASYMMETRIC - lambda
scales acceleration only. Capping braking with the same number limited deceleration to
11.9 m/s^2 and cost cones and off-tracks; locking under braking is the slip-ratio envelope's job.

**Beware small samples - I got this wrong twice tonight.** ProgressMPC at its best settings is
about **2 clean runs in 6**. A 0/3 result is inside binomial noise of that, and I twice declared
a "regression" from a 3-run comparison and changed code in response. Use 6+ repeats before
believing any reliability difference.

**Instrumentation lessons (both cost a full experiment):**
- The slip-ratio early kill first fired on the STANDING START - slip is `(r*omega - vx)/vx`, so
  it saturates to 1.0 at vx~0 for any wheel rotation. Now gated on speed > 3 m/s.
- At 0.5 it then fired on NORMAL driving, abandoning configurations that had produced clean
  25.5 s laps. Now 0.9 sustained 1.5 s, and `slip_max` / `slip_frac_over_05` are RECORDED per run
  so the threshold is checkable rather than assumed. Note the recorded fraction includes braking
  (the metric takes `abs`), so it is not purely a wheelspin measure.

**Solve time is not the problem** when the machine is quiet: 10-11 ms mean at load ~3. Earlier
readings of 23 ms were taken at load 12-17 (Foxglove + Firefox), and a 6.8 ms reading came from a
quiet machine at load 0.4. Same binary throughout - the desktop load explains all of it.

## SESSION 3 (2026-08-14) — mechanism FOUND, two model fixes tried

**The departure mechanism is traced, not inferred** (`departure_trace.py`, data in
`mpcc_data/departures/`). Approaching every departure:

    early-pre  err 0.10->0.20  speed 3.4->5.0   |throttle| 0.61  slip 1.00  t_tot 8.8  sqp 1.0
    late-pre   err 0.22->2.00  speed 5.2->11.8  |throttle| 0.80  slip 0.84  t_tot 8.4  sqp 1.0

The car is wheelspinning at 100% from 3 m/s onward, BEFORE the tracking error grows. Commanded
throttle is 0.61-0.80; the traction limit is mu*m*g*rear_fraction / (gear*eta*T_max) ~= 0.43.
`t_tot` is flat at ~9 ms and `sqp_iter` never moves, so **the solver is not involved** - solver
failures are a symptom of a run going wrong, never a cause. Do not chase them.

**Wheelspin fraction predicts the outcome almost perfectly:** every run under 5% of the lap in
slip finished clean, every run over 30% failed. Use `slip_frac_over_05` (recorded per run) as the
tuning signal - it is continuous, unlike the binary clean/not-clean that made 72 runs
uninformative.

**A throttle cap PROVES the mechanism but is the wrong fix.** Capping commanded throttle at 0.45
gave 3/6 clean, 5/6 completed (vs ~2/6), err 0.122, and two runs with ZERO wheelspin - a first.
The user rejected it on principle, correctly: the vehicle model should decline the throttle, not
an external bound. Config knob `progressmpc_max_throttle` exists and defaults to 1.0 (disabled).

**Quasi-static wheels (index-1 DAE) - TRIED AND REVERTED, measurably worse.**

| attempt | NaN/run | solver failures | result |
|---|---|---|---|
| plain DAE | 2532 | - | unusable |
| damping on wheel speed 1.0 | 283 | 72-317 | 0/3 |
| damping on wheel speed 5.0 | 0 | - | model wrong: 486 N m drag vs 391 N m available |
| damping on SLIP speed 50 | 19 | 31-49 | 0/3 |
| slip damping + known-good weights | - | 17-112 | **0/3 vs 2-3/6 for the ODE** |

Root cause is structural: past the Pacejka peak there is NO equilibrium (a real wheel keeps
accelerating), so the algebraic solve has no root exactly where the car wants most torque.
Backup of the working files was kept at `scratchpad/prequasi/` and used to revert.

**The cost weights had silently become a TRACKER.** `[progress 0.05, n 30, theta 20]` is a lateral
weight 600x the progress weight - the opposite of this controller's purpose. It came from a
diagnostic that deliberately made it a tracker to check the Frenet plumbing, and it stuck as the
baseline; in effect the lateral weight was propping the car up to mask the wheelspin. Design
intent is progress-dominant with just enough lateral weight to keep the QP well-posed.

**Corridor geometry:** the car is off track past ~1.5 m of centreline deviation, but the CAR IS
1.2 m WIDE, so usable deviation is ~0.9 m. Correct range is 0.6-0.9. Tracking error is NOT a
quality metric for this controller - it only matters past ~1.5 m.

**AGREED NEXT (option 1): force as the control input.** Command longitudinal force (proportional
to torque) instead of throttle, with the friction ellipse as an explicit constraint and Ax/Ay
limits as the tuning knob. The wheel dynamics leave the problem entirely - no stiff mode, no
inertia to get wrong - and the traction limit lives in the model's own constraint set, which is
what the user asked for. Throttle for the plant follows from
`throttle = Fx*r / (gear*eta*T_max)`.

**Build hazards that cost hours this session:**
- `pkill -f <pattern>` where the pattern appears in your own command line kills your own shell.
  Happened five times. Use explicit PIDs.
- `-O3 -march=native` TUs exceed the ~4 GB free with Foxglove/Firefox up; the OOM killer takes
  the whole process tree. Build detached with a retry loop, or `-j1`.
- Deleting `CMakeFiles/<target>.dir` removes CMake's generated `build.make`, and colcon will not
  regenerate it - you need `--cmake-force-configure`.
- After changing NX, stale object files smash the stack at runtime. Force a full recompile.
- Two sweeps running at once corrupt each other. Verify nothing is running before launching.

## SESSION 3b — FORCE-INPUT MODEL (option 1) IS IMPLEMENTED AND BUILDS

`progressmpc_acados.py` now commands longitudinal FORCE, not throttle. NX 16 -> 12, NU 2, NH 5.
Backup of the throttle-input version: `scratchpad/preforce.py` / `preforce.cpp`.

- States: `(S, N, THETA_E, VX, VY, YAW_RATE, AX, AY, STEER, FX_CMD, STEER_CMD, FX_APPLIED)`.
  The whole powertrain, differential and wheel-dynamics block is deleted - no wheel states, no
  stiff mode, no inflated inertia anywhere in the problem.
- **Lateral Pacejka is fully intact** (per wheel, with vertical load and slip angle). Only the
  longitudinal branch is gone (`fx_* = 0`). NOTE: with fx=0 the tyre-level combined-slip
  reduction of lateral grip is no longer applied; that coupling now lives in the chassis-level
  envelope instead. They are not equivalent - the envelope acts on all four wheels at once.
- Constraint set: `[slip_rl, slip_rr, acceleration_envelope, corridor, power_demand]`.
- **The envelope is LOAD-PROPORTIONAL**, which matters a lot:
  `longitudinal_capacity = tire_D * (rear_axle_load + brake_blend * front_axle_load)` and
  `lateral_capacity = tire_lateral_D * total_axle_load`. It therefore tightens under braking,
  widens on corner exit and widens with downforce, exactly as Pacejka did. `brake_blend` is ~1
  when decelerating and ~0 when accelerating, because only the rear axle drives but all four
  brake. A constant limit threw all of this away and permitted 9.8 m/s^2 on a rear axle that can
  only deliver ~6.9.
- Output conversion: `throttle = Fx * r / (gear * eta * T_max)`, constant `kForceToThrottle`.
- Power constraint `Fx * v <= P_max * eta` is required: force alone cannot express that the motor
  delivers less at speed.

**Trap that cost a round:** `u[0]` is a FORCE rate now. The bound was still `max_throttle_rate =
4.0`, i.e. 4 N/s, so reaching full drive force would take 15 minutes - four silent no-lap runs
with ZERO solver failures. It is now `max_throttle_rate * max_drive_force`, and the cost residuals
for force and force-rate are normalised by `max_drive_force`.

**Current results (progress-dominant weights [0.2, 0.5, 2.0], corridor 0.8):**

| lambda (both) | clean | mean wheelspin fraction |
|---|---|---|
| constant limits, 0.70 | 1/4 dirty | 0.20-0.29 |
| load-proportional, 0.90 | 0/4 | 0.141 |
| load-proportional, 0.75 | 1/4 (60.6 s, 7 cones) | 0.124 |

Wheelspin is roughly halved versus the constant-limit version, and one earlier run reached
`slip_max 0.036` - essentially zero, which the throttle-input model never achieved. But it is not
clean yet and completed laps are slow.

**Next steps:** sweep lambda_lat and lambda_long INDEPENDENTLY (they were moved together above,
which confounds them), then the progress/lateral weights and corridor 0.6-0.9 on top. Judge on
cones, completion and wheelspin fraction - NOT tracking error, which is expected to rise for a
controller choosing its own line.

## SESSION 3c — COMBINED-SLIP BUDGET WORKS; IT IS NOW UNDER-DRIVEN

The longitudinal limit is now taken from the friction circle, per wheel, using Pacejka's own
lateral force and the model's load transfer:

    Fx_budget = D_x*Fz * sqrt(1 - (Fy / (D_y*Fz))^2)

summed over the rear axle under power and all four wheels under braking (`brake_blend`), then
scaled by lambda_long. A tyre at its lateral limit has zero longitudinal budget; one running
straight has all of it. This is the constraint that a fixed acceleration ellipse cannot express.
Verified: D_x = tire_D = 1.398, D_y = tire_lateral_D = 1.703, and `fy = -Fz*D_y*sin(...)` so the
`fy/(D_y*Fz)` normalisation is correct and bounded to +-1.

**Result: 42.74 s, 2 cones, err 0.388, ZERO solver failures.** A completed, nearly clean lap - the
force-input model with the combined-slip budget drives. Note an earlier identical-config run
reported 4/4 no-lap; that was a stale binary (make succeeded, install had not propagated). ALWAYS
confirm the installed binary before believing a result - check the envelope log line.

**Open problem: it is too slow.** vmax 11.5 m/s against ~22 for the throttle-input baseline. The
budget permits ~2450 N at standstill (~10 m/s^2) and the car does not spend it, so the constraint
is not the limit - the COST is. Prime suspect: when force replaced throttle the residuals were
normalised by `max_drive_force`, but their weights were carried over from throttle units
unchanged (force 0.05, force-rate 2.0). The rate residual now spans +-4, so a weight of 2.0
damps the force demand hard. Rescale those two weights first, then raise lambda.

## THE OPEN LEAD — start here

**Reliability, not pace.** ~2 clean runs in 6 at the best known settings. Best clean lap 25.5 s
against mpczinho's 24.68 s on the same binary; the 23 s target is not close and will not be
reached by tuning a controller that finishes a third of the time.

The most promising structural fix is the **quasi-static wheel model**: drop the four wheel-speed
states and solve slip algebraically. That removes the stiff mode entirely rather than
approximating around it, which is what forces the inflated inertia and its traction optimism. It
also removes 4 states from the QP. Untried, and the natural next step.

Second option if that is too invasive: accept the optimism and bound longitudinal demand with the
`AX` state bound (already implemented, `progressmpc_grip_utilisation_longitudinal`), then sweep it
at 6+ repeats. It was never swept at adequate sample size because two rounds were lost to
instrumentation faults.

## Bugs found and fixed (do not reintroduce)

1. **Vacuous corridor.** `local_path_resampled_with_spline` anchors its first point ON THE CAR, so
   `n ≡ 0` and `θ_e ≡ 0` every cycle — the corridor constraint was measuring from wherever the car
   happened to be and it drifted 10 m off without noticing. ProgressMPC now does its own
   arc-length resampling anchored on the car's **projection onto the path**
   (`progressmpc.cpp::set_path_in_solver`). Do not switch back to the shared helper.
2. **Phantom curvature.** Tangents taken from the planner's original segments (~0.1-0.3 m apart)
   are mostly quantisation noise; differentiated into curvature they saturated the ±0.5 clamp and
   the controller sat at **full steering lock** chasing corners that did not exist. Tangents are
   now computed from the resampled points over a ≥0.5 m baseline.
3. **Cost scaling inverted the priorities.** At `progress_ceiling=45` and weight 10 a stage cost
   ~16000 against a ~10000 corridor violation — leaving the track was the cheaper option. Progress
   weight must stay well UNDER the constraint slack penalties.
4. **Friction ellipse was unusable.** Built from the squared Pacejka force sum it drove the QP to
   ACADOS_MINSTEP on most cycles. Now two separate LINEAR bounds: lateral `vx*yaw_rate`,
   longitudinal `vx_dot`. Rectangular rather than elliptical budget — slightly optimistic where
   braking and cornering combine. Revisit only once reliable.
5. **Slip-ratio envelope was missing** (user's suggestion). Adding it produced the first clean lap.

## Harness notes

- `supermpc_sweep.py` now early-kills on driven-wheel slip ratio > 0.5 sustained 0.4 s
  (`--max-slip-ratio`), alongside cones / off-track / lap time.
- Fresh `rclpy.context.Context()` per case — the global context cannot be re-inited after
  shutdown, and one shared across a sweep goes invalid part-way through. Rows stream to CSV per
  run so a crash cannot lose completed work.
- Sim video driver is now `dummy` by default (`SWEEP_SDL` to override). Measured: no solve-time
  difference vs x11, so this is only about not opening windows.
- **The sweep scripts rewrite `config/control/invictasim.yaml` and `config/planning/invictasim.yaml`
  per case** and restore on exit. Do not drive the car manually while one is running.
- Do not `pkill -f node_control` while a build is running — that string is also the compiler's
  target directory name and it kills gcc mid-build.

## Solve time

Profiled with `solve_profile.py`. The "3 ms vs 14 ms" discrepancy was a measurement artefact:
3 ms is the **standing start**; settled median is 6.8 ms with **p90 19 ms, max 26 ms** against a
25 ms command period. Heavy-tailed, not shifted. Budget is tight — prefer fewer stages over a
longer horizon rather than more stages.

## Next steps, in order

1. Reliability is the blocker, not pace: ~2/6 clean at best, and 23 s is far off (best clean lap
   25.5 s, mpczinho 24.68 s). The most promising structural fix is the quasi-static wheel model.
2. Re-run the lateral λ survivors at 6+ repeats to settle 0.60 vs 0.70 properly. Three repeats
   cannot separate them - the between-repeat spread exceeds the between-λ difference.
3. Then round 2's remaining axes (`progress_cases2.txt`), which sweeps the newly split
   longitudinal budget independently of lateral.
4. Consider splitting the longitudinal budget again into separate acceleration and braking
   limits — it is currently symmetric, so it caps braking as hard as it caps acceleration.

---

## SESSION 4 — FOUR REAL BUGS FIXED; WHEEL-MODEL QUESTION STILL OPEN

### Bugs fixed, each confirmed from data (not inferred)

1. **Frenet frame latching.** The projection was a global `argmin` over every path point with no
   continuity, so wherever the closed track passes near itself — or whenever the car ran wide —
   the frame snapped to a different lap section. Traced: `n` moving >3 m in one control period
   (physically impossible) and `theta_e` flipping to ±π. Fixed with a 6 m geometric window plus a
   heading filter, tangent computed from geometry over a 20-point baseline (NOT
   `points[i].orientation`, which is untrusted at this spacing — filtering on it drove the global
   fallback 313 times in one run and silently disabled the whole window).
   **Effect: solver failures 9–30 → 1–2 per run.**

2. **Slack pricing wrong in kind.** Both linear (`zl`) and quadratic (`Zl`) were set to the same
   value. A linear slack penalty is an EXACT penalty — above a finite threshold the solution
   equals the hard-constrained one — while a quadratic one is inexact and must be made enormous
   to bite. 1e5 on the Hessian diagonal next to cost weights of order 1–50 destroys conditioning.
   Split to `zl` large / `Zl` = 1e-2·that. **Effect: solver failures 15–18 → 0.**

3. **Tyre limit was cheaper than driving slowly.** `envelope_penalty` 1e3 meant a FULL traction
   violation cost ~2000 while the progress residual cost ~2900/stage at w=10. Measured: 2681 N
   commanded against an ~1818 N rear budget, then full opposite lock. Raised to 1e5. NOTE: the
   file's own comment documented this ordering requirement, and raising the progress weight
   0.2 → 10 → 50 silently invalidated it. Much of the earlier weight tuning was measuring how
   badly traction was being ignored.

4. **Planner speed was a hard cap.** `h_e = vx - v_max` used the planner's own speed at the
   horizon end, so with a conservative smoothing margin the car could never exceed 8–17 m/s. The
   controller was a planner-speed follower — the exact thing this design exists to avoid. Now
   derived from geometry: `v ≤ sqrt(lambda_lat·a_lat_max/kappa)`, peak kappa over the last 5
   stages.

Also: `progressmpc_max_slip_ratio` and `progressmpc_max_throttle` were dead keys (read into
unused variables — tuning them did nothing); a spurious `ERROR: first point doens't match state`
fired on nearly every solve; ~14 dead constants and a docstring describing SuperMPC.

### What limits pace — MEASURED, use this

Limiter probe, one knob at a time, 4 repeats each (force-input model, physics carries over):

| case | completed | vx median | best lap |
|---|---|---|---|
| base (lat .8, long .8, prog 10) | 4/4 | 5.60 | 46.76 s |
| lambda_lat 1.0 | **0/4** | 4.76 | — |
| lambda_long 1.0 | 3/4 | 5.57 | 46.71 s |
| **progress 50** | 2/4 | **6.74** | **35.23 s** |

**Pace lives in the COST, not the grip budgets.** lambda_lat 1.0 plans 17 m/s² (1.73 g) the tyres
cannot deliver → every run departs; real capability ≈13.6 m/s². lambda_long does nothing. Do not
re-sweep these two — it is 40 minutes to re-derive a known answer.

Untried and structurally the largest lever: **corridor half-width**, still at 0.5 m. Usable is
~0.9 m (track half-width 1.5 − car half-width 0.6). Corner speed goes as
`v = sqrt(lambda_lat·a_lat/kappa)` and kappa is set by the LINE — a wider corridor lets the car
straighten the corner. At 0.5 m this is a centreline tracker wearing a corridor's clothes.
`pace_sweep.sh` is written and ready for this.

### Graded time grid — works as a mechanism, has NOT produced a healthy solver

Implemented per the "small first steps" idea, and verified in the generated solver
(`build/control/control/progressmpc/acados/acados_ocp_mpc.json`): 40 steps, 2.0 ms → 64.2 ms cap,
sums to tf exactly, `nx=16`, per-stage `sim_method_num_steps`.

Wheel-slip mode: `d(kappa_dot)/d(kappa) = -r²·Fz·B·C·D/(I·V) ≈ -2900 1/s`, so **tau ≈ 0.34 ms**.

| configuration | NaN/run |
|---|---|
| geometric 2 → 323 ms (unbounded ramp) | 94 |
| **capped 2 → 64 ms, uniform 2 substeps, newton 5** | **14 ← best** |
| capped + per-stage substeps floor 1, newton 5 | 77 (floor 1 halved the fine end — regression) |
| capped + per-stage substeps floor 2, newton 8 | 22 |

**More substeps and more Newton iterations made it WORSE.** The failure is therefore not
integration accuracy. Physical reading: the sim shows slip ratio reaching **8.2**, so the wheels
genuinely spin ~9× road speed; past the Pacejka peak `dFx/dkappa` goes small and flips sign, so
Newton has no gradient. The trajectory being integrated is itself unphysical — no grid fixes that.
The stiffness and the wheelspin are the same phenomenon.

**COUPLING BUG the graded grid exposed (would have been invisible):** the path resampler sampled
at uniform arc length while the shooting grid was graded, so stage *i* was handed the geometry of
a point the car reaches at a completely different time — every per-stage curvature and corridor
value describing the wrong part of the track, silently. Both grids now derive from
`first_step_seconds`/`step_growth_ratio` identically (`graded_time_steps` in the .py,
`graded_step_fractions` in progressmpc.cpp). **If you change one, change the other.**

### Open question and the two honest options

True inertia 0.2/0.4 has not produced a completed lap. In progress when context ran out: measuring
3× (0.6/1.2). At 3×, tau ≈ 1 ms against a 2 ms first step, so the model still SEES spin-up and the
slip-ratio constraint still has meaning — which is the property that actually matters, versus the
old 10× fudge that made the model blind and caused it to command ~2× the throttle the tyres could
take. `py_true_inertia.py` in scratchpad holds the true-inertia version.

If 3× also fails, the fallback is the **force-input model** (`force_model_final.py/.cpp` in
scratchpad): 4/4 completions, 0 solver failures, 0.26 m tracking error — the most reliable this
controller has been — but slow (46 s, vx median 5.6). It has no wheel states at all, so the
stiffness question disappears entirely.

### Traps for whoever picks this up

- **The sweep's `vmax` column is not body speed.** It reports >60 m/s, impossible for this car.
  Read `vx=` from the controller log instead. Any prior pace conclusion resting on `vmax` is
  suspect.
- **Builds OOM** at `-j2` with `-O3 -march=native`. Use `MAKEFLAGS=-j1`. A `Killed` in the log is
  the OOM killer, not a code error.
- **Never write a wait-loop as `until ! pgrep -f "colcon build"`** — the pattern matches the shell
  running the loop, so it waits on itself forever. Cost a build launch this session.
- `supermpc_sweep.py` now hard-fails on a stale install (`--allow-stale` to override). This had
  already invalidated a full 60-run dataset earlier in the campaign.
- Judge on clean-lap RATE, not best lap. A fast time in 1 of 6 is noise; this campaign was misled
  by exactly that twice.

### Slip-ratio bound 0.10 — TRIED, does not fix the numerics (and the reason matters)

Pacejka longitudinal peak is at **kappa = 0.1235**:

| kappa | mu | dmu/dkappa | branch |
|---|---|---|---|
| 0.10 | 1.395 | +0.36 | stable |
| 0.15 | 1.396 | -0.11 | **past peak** |

So 0.15 does sit past the peak, and as a CONTROL setting 0.10 is better founded - it holds the
tyre on the rising branch for 0.2% of peak grip. Worth keeping for that reason alone.

But it does NOT fix the NaNs. Measured at identical solver settings, true inertia:

| slip bound | NaN/run | vx median | vx max |
|---|---|---|---|
| 0.15 | 22 | - | - |
| 0.10 | **28** | 0.22 m/s | 5.26 m/s |

Slightly worse, and the car barely moves.

**Why - remember this.** A SOFT constraint shapes the QP's cost; it does not restrict what the
INTEGRATOR traverses. Forward simulation still passes through the high-slip region for a given u,
so Newton still loses its gradient there. Constraining slip changes what the optimizer PREFERS,
not what the integrator VISITS. Any future fix for the NaN problem must act on the state
trajectory (integrator side), not on the constraint set - e.g. bounding the wheel-speed STATE
(lbx/ubx on W_RL/W_RR relative to vx), which the integrator does respect, rather than penalising
slip in the cost.

The car being strangled at 0.10 (median 0.22 m/s) also says the slack penalty now dominates the
progress term at these cost weights. If 0.10 is kept for control reasons, the progress weight has
to come up with it.

**Best measured configuration of this model line: slip 0.15, uniform 2 substeps, newton_iter 5 -
14 NaN/run. Nothing since has beaten it, and the tree is left at exactly that state.**

---

## SESSION 5 - build infrastructure, and a correction

### The OOM was never the build's fault

Two builds were running concurrently. `colcon build --packages-up-to control` (started outside
this agent session, PID 433233) overlapped with the agent's `--packages-select control`. Each
spawns its own `cmake --build`, and `-O3 -march=native` on this codebase costs several GB per
translation unit. On a 14 GB box with ~2 GB free and 11 GB already held by the IDE, agent
runtime, and foxglove-studio, two concurrent `cc1plus` processes are enough to trigger the OOM
killer. The build log just stops mid-target with no error - that truncation IS the OOM
signature, so do not go looking for a compile error when a build "fails" with empty stderr.

**Rule: check `pgrep -af colcon` before starting a build.** `MAKEFLAGS=-j1` does NOT protect
against this - it limits jobs WITHIN one make invocation, and the notice
`jobserver unavailable: using -j1` confirms sub-makes are already serial. It cannot serialise
two independent colcon runs.

Secondary saving: `-DBUILD_TESTING=OFF` drops the `control_test` target, which otherwise
compiles concurrently with `node_control` inside a single colcon run.

### Correction: the stale-install guard is NOT broken

An earlier note in this session claimed the guard had a hole and had allowed a measurement
against a stale binary. **That was wrong.** The guard fired correctly:

```
BUILD_EXIT=2
STALE INSTALL: 2 source file(s) are newer than the installed binary
```

The failed build left the binary older than the sources, `verify_install_fresh()` detected it,
and the sweep refused to run. `verify_install_fresh()` in `supermpc_sweep.py` needs no fix. It
has now caught a stale binary in production, which is exactly the failure that invalidated a
60-run dataset earlier in this campaign.

### State of the tree

The curvature-derived terminal speed cap has been REVERTED to the planner-speed cap.
Reason: it measured 0/6 completions against a ~2/6 baseline and was kept anyway on the
reasoning that "it only survives if the projection fix rescues it". That reasoning was never
validated, and the per-stage corner-speed constraint that would have justified the cap existed
only in the force-input model and was never carried into the restored Pacejka model - so the
cap lifted the speed limit on straights with nothing left to command braking for corners.
Keeping a change that its own measurement contradicted is the single clearest process error of
this campaign.

**Not yet confirmed: whether the revert restores lap completion.** The measurement was still
running when this was written. Do not assume it worked - read `revert3.csv` first.

### If the revert does not restore ~25.9 s

Bisect the remaining deltas from the 25.95 s baseline, one at a time. All were tuned against
the FORCE-INPUT model, not the restored Pacejka model, so none carries evidence here:

1. `envelope_penalty` 1e5 (was 1e3)
2. `corridor_penalty` 2e4 (was 5e3)
3. the L1/L2 slack split (`Zl = 1e-2 * zl`)
4. the Frenet projection change

Known-good fallback in scratchpad: `force_model_final.py` / `force_model_final.cpp` -
4/4 completions, 0 solver failures, 0.26 m tracking error, 43-46 s. Slow, but it drives.

---

## SESSION 6 - the regression was the COST WEIGHTS, in config

Bisected properly this time, one layer at a time, measuring each.

| change tested | rebuild | solver failures | laps |
|---|---|---|---|
| terminal cap -> planner speed | yes | 10-91 | 0/5 |
| + wheel inertia 0.2/4.0 -> 2.0/2.0 | yes | 9-84 | 0/5 |
| + restore whole preforce.py solver | yes | 9-40 | 0/5 |
| + restore documented config weights | **no** | **0, 0, 0, 0, 0** | 0/5 (never moves) |

**The solver file was never the regression.** Restoring the entire known-good `preforce.py`
changed nothing. The failures lived in `config/control/invictasim.yaml`, and the single most
damaging entry was the cost-weight vector.

Broken config was `[1, 0.05, 0.5, 1.0, 0.02, 0.1, 5.0, 15.0]` against a design intent of
`[0.2, 0.05, 0.5, 6.0, 0.05, 0.3, 2.0, 4.0]`. The `vy` weight at 1.0 instead of 6.0 is the one
that matters - it is the sideslip regulariser, and with it 6x too low the car had almost nothing
resisting rotation. The logged trace showed exactly that: `theta_e` reaching -1.46 rad (-84
degrees) with steering pinned at its +-0.335 limit, speed lurching 0 -> 7.77 -> 0.18 m/s, and the
Frenet reference heading latching to -2.678 rad once the car was sideways and never recovering.

The ref_yaw latch is a CONSEQUENCE, not a cause. Do not go chasing the projection code for it -
the projection is being asked to localise a car that is pointing 84 degrees off the track.

### Two real defects found on the way, both worth keeping fixed

1. **Wheel inertia had drifted to front 0.2 / rear 4.0.** SuperMPC - the robust controller,
   sharing this wheel subsystem line for line, same `wheel_speed_scale`, same velocity floor,
   and likewise NO lbx/ubx on the wheel speeds - uses 2.0 / 2.0. The inertia is therefore the
   only thing holding either integrator together. Front sat at the true uninflated value that
   the comment directly above it argues cannot work, rear at twice SuperMPC's: a 20:1 ratio,
   unphysical and stiff. Symptom was rear wheels at 22.9 rad/s against a body speed of
   -0.85 m/s and the predicted trajectory diverging past +-650 m/s by stage 7. Now 2.0 / 2.0.
2. **`-DBUILD_TESTING=OFF` fixes the build OOM.** It drops the `control_test` target, which
   otherwise compiles `progressmpc.cpp` concurrently with `node_control`. Build now takes
   4 min 39 s and does not get OOM-killed. Combine with checking `pgrep -af colcon` first.

### The car will not launch at progress weight 0.2

Healthy solver, zero failures, and `vx = 0.00` for the entire run. This is pricing, not a bug.
At standstill the progress deficit is the whole ceiling, so a stage costs `w * 30^2 = 900w`,
while the standing-start slip-ratio violation costs `envelope_penalty = 1e3` and a corridor
violation `corridor_penalty = 5e3`. That brackets the weight:

    900w > 1e3  ->  w > 1.1    below this the car never moves (measured at w=0.2)
    900w < 5e3  ->  w < 5.5    above this leaving the track is cheaper than staying on it

So the usable window is ~1.5-5, which is being swept now. Note the broken config's w=1 sits just
under the launch threshold, and its w=10 predecessor sat far above the corridor threshold - the
two failure modes on either side of a fairly narrow window.

**The proper fix is the separate `slip_ratio_penalty` (1e3, below `envelope_penalty`)** that
session 4 added and the preforce restore removed. It makes the standing start feasible without
having to buy motion with a large progress weight, which widens the window at both ends.
Re-apply it once a progress weight is confirmed to drive.

### CORRECTION to the section above - the weight conclusion was WRONG

The "solver failures 0/0/0/0/0" result that the cost-weight conclusion rests on was measured
against a **corrupted binary** and must be discarded. So must the launch-pricing analysis that
followed it.

**`progressmpc_prediction_horizon_steps` is a BUILD-TIME parameter, not a runtime one.** The
generator calls `load_mpc_parameters()` and bakes `N_horizon` into the generated C. But
`progressmpc_acados.cpp` indexes stages from BOTH sources - the config value at four call sites
and `nlp_dims_->N` at four others. Change the YAML without rebuilding and they disagree
silently: acados solves its own consistent N-stage problem and honestly reports **zero
failures**, while the path and parameters land on the wrong stages. The car just sits still.

That is what "0 failures, vx = 0.00" actually meant. Three measurement cycles were spent on it,
including a whole progress-weight sweep and a confident piece of arithmetic about launch
pricing, all of it worthless. **The car launches fine at progress weight 0.2.**

Guard added at the top of `initialize_solver()`: it compares the configured step count against
`nlp_dims_->N` and logs a HORIZON MISMATCH error naming the rebuild requirement. It does not
silently "fix" the value, because the other call sites still read the config - the only correct
response is to rebuild.

**Rule: `progressmpc_prediction_horizon_steps` and `progressmpc_prediction_horizon_seconds`
require `colcon build` before they mean anything. Cost weights, corridor half-width, lambda,
slip bounds and throttle cap are genuine runtime parameters and do not.**

### First VALID baseline of session 6

preforce solver + inertia 2.0/2.0 + weights `[0.2, 0.05, 0.5, 6.0, 0.05, 0.3, 2.0, 4.0]`,
N=30 built and configured consistently, corridor 0.5, lambda_lat 0.70, slip ratio 0.15:

| metric | value |
|---|---|
| completions | 0/5 |
| solver failures | 5, 16, 5, 38, 18 |
| track error | 9.4 - 13.4 m |
| vx median / max | 3.75 / 13.55 m/s |

The car drives properly now. Failure mode is a lateral divergence: it reaches 13.55 m/s within
the first 2 s, then `n` oscillates with growing amplitude - 0.34, -0.87, 0.97, -1.98, 3.45,
-11.34 - with steering saturated at +-0.335 throughout. It arrives at the first corner far too
fast. Sweeping corridor width (0.5 is tighter than the documented usable 0.6-0.9) against
lambda_lat, both true runtime parameters.

### The structural difference from SuperMPC: WHERE the speed cap lives

This is the most likely remaining cause and the best lead for the next session.

**SuperMPC constrains speed at EVERY stage.** Its cost vector carries
`speed_excess = smooth_relu(x[VX] - p[P_V_CAP])` weighted **40** - the heaviest entry in the
whole vector, deliberately outweighing both of its ambition terms so the cap actually binds.

**ProgressMPC constrains speed at ONE stage.** `p[P_V_MAX]` appears only in `con_h_expr_e`, the
terminal constraint. Nowhere in the stage cost or the stage constraints is there anything tying
speed to the path.

The consequence is a classic receding-horizon failure. The optimizer can be flat out for stages
0..N-1 and still satisfy a terminal cap by planning a hard brake in the last few stages. Only
u[0] is ever applied, so that brake is re-planned every cycle and never executed. The measured
behaviour matches exactly: 0 to 13.5-16.6 m/s within the first 2 s, then departure at the first
corner.

**Why the lateral grip budget does NOT save it.** `grip_budget` bounds the ACTUAL lateral
acceleration, so it is reactive: on the straight approaching a corner a_lat is ~0 and the
constraint is slack. It only bites once the car is already in the corner and committed at a
speed it cannot carry. Anticipation has to come from the horizon covering the corner, and at
16 m/s a 2.0 s horizon reaches only 32 m.

Two candidate fixes, in order of preference:

1. **Per-stage corner-speed constraint** from the curvature already available as P_KAPPA:
   `vx^2 * |kappa| <= lambda_lat * max_lateral_acceleration` at every stage. This is the lateral
   grip budget applied KINEMATICALLY rather than reactively, so it anticipates. Physically
   principled, needs no new parameter, and P_KAPPA is already per-stage. Note a version of this
   existed in the force-input model and was never carried into the Pacejka model - that omission
   is also why the curvature-derived terminal cap measured 0/6 and was reverted.
2. **Lengthen the horizon** so it actually covers the corner at racing speed. Cheaper to try but
   treats the symptom, and horizon steps is BUILD-TIME - it needs a rebuild to take effect.

Do NOT reach for cost weights again before trying (1). The lateral divergence with saturated
steering is what a car looks like when it is simply carrying too much speed into a corner, not
what a mistuned regulariser looks like.

### Corridor x lambda sweep - no clean lap anywhere

| case | corridor | lambda_lat | completed | cones | solver failures |
|---|---|---|---|---|---|
| w07l70 | 0.7 | 0.70 | 0/3 | - | 6, 6, 28 |
| w09l70 | 0.9 | 0.70 | 0/3 | - | 3, 13, 32 |
| w07l60 | 0.7 | **0.60** | **2/3** | 6, 14 | 3, 9, 38 |
| w09l60 | 0.9 | **0.60** | 1/3 | 22 | 10, 16, 20 |

**Corridor width does nothing.** 0.7 and 0.9 are indistinguishable at either lambda. The
documented "usable 0.6-0.9" range is geometrically true but is not the binding limit, so stop
spending runs on it.

**lambda_lat 0.70 -> 0.60 is the only axis that moved completion**, 0/6 to 3/6, confirming
session 1's finding that lambda dominates. But every completed lap is filthy - 6, 14 and 22
cones - so the car is getting round by cutting the course, not by driving it.

Two cautions for whoever picks this up:

* **The 10.58 s lap is not real.** The benchmark is 24.68 s and the best this controller has
  ever driven is 25.5 s. A 10.58 s lap with 6 cones means the lap detector triggered without
  the car completing the circuit. Do not record it as a pace result, and treat lap times from
  runs with high cone counts as suspect generally.
* **`avg_track_err` is low (0.57-1.45 m) on exactly the runs with the most cones.** Those two
  facts together say the metric is being computed against whatever centreline the car is nearest,
  not against the course it was supposed to drive. Track error remains not a quality metric here.

Net: the controller drives, but it cannot complete a clean lap at any point in the
corridor x lambda plane. That is consistent with the speed-anticipation diagnosis above - lower
lambda merely slows the car enough to survive corners it still enters without planning for. The
per-stage corner-speed constraint is the next thing to build, and nothing in this sweep argues
against it.
