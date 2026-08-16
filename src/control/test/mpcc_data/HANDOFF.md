# SuperMPC MPCC work — state at pause

Stopped mid-sweep. Everything below is measured, not inferred; where something is unproven it
says so.

## Current repo state

**Code changed (uncommitted):**

| file | change |
|---|---|
| `include/solver/supermpc_acados/supermpc_acados.py` | MPCC progress state added |
| `src/solver/supermpc_acados.cpp` | progress arc-length param + state init |
| `config/control/invictasim.yaml` | 11-entry weight vector, horizon 25 |
| `src/control/CMakeLists.txt` | acados codegen `.so` declared as OUTPUT (unrelated bug fix) |
| `compile.sh`, `car_compile.sh`, `setup_shortcuts.sh` | Ninja → Make `-j2` (unrelated, fixes OOM) |
| `src/planning/src/planning/smoothing.cpp` | 2 bug fixes only (degenerate normal, osqp_setup arg order) |

**Built and working.** Planning is plain minimum curvature — all lap-time/curvature-limit work
was removed on request.

## What MPCC changed

- `NX 16→17` (`PROGRESS`), `NU 2→3` (progress rate `u[2]`), `N_PARAMS 6→7` (`P_S_REF`)
- Reference point slides along the path tangent by `PROGRESS − P_S_REF`, so contouring error is
  unaffected by timing and lag absorbs it
- Progress reward written as residual `(v_cap − ṡ)` because a linear reward is not expressible
  in `NONLINEAR_LS`
- Weight order: `[contour, heading, lag, ambition, excess, progress, vy, throttle, steer,
  thr_rate, str_rate]`
- `ambition > 0` = planner speed is a rough target; `progress > 0` = planner speed is a ceiling

**Progress weight must never be 0** — it is the only cost term acting on `ṡ`, so at 0 the QP is
singular in that direction and hits `ACADOS_MINSTEP`.

## THE OPEN BUG — start here

**The controller brakes itself to a standstill.** From `trace_withcmd_0.csv`:

```
v=4.16  target=10.95  err=1.33 | throttle=-0.020   <- braking
v=4.01  target=11.10  err=1.76 | throttle=-0.090
v=3.89  target=11.11  err=2.00 | throttle=-0.098   <- braking harder
```

Negative throttle while 7 m/s below target, braking harder as error grows. `trace_1.csv` catches
the endpoint: position frozen, `v=0.00` for several consecutive samples while commanded 9.9 m/s.

**Mechanism (hypothesis, not yet confirmed by a fix):** at `contour=80` vs `progress=1` (80:1),
once the car is off the line the cheapest way to stop contour error accumulating is to stop
moving. Braking is locally optimal; the reference keeps advancing; error grows; braking
increases.

mpczinho is immune because its speed comes from a separate PID that cannot see contour error and
therefore cannot trade speed away for it.

This explains every anomaly in the earlier data: departures correlating with no parameter, both
controllers failing, lap time pinned across a 6.7x weight swing, and the 40 ms test changing
nothing.

**Second candidate fix, untested:** `path_stager.cpp` clamps the reference speed scale to
`[1.0, 1.8]`, with a comment explaining that scaling below 1.0 removes the signal telling a slow
car to speed up. With MPCC that reasoning no longer holds — the progress term now provides that
signal — so allowing the scale below 1.0 would stop the reference running away from a slow car.

## Measured results

**Head-to-head, matched load (`deadline.csv`):**

| arm | clean | lap [s] | err [m] |
|---|---|---|---|
| mpczinho 25 ms | 1/3 | 25.07 | 0.132 |
| supermpc 25 ms | 2/3 | 25.85 | **0.074** |
| mpczinho 40 ms | 2/3 | 25.07 | 0.132 |
| supermpc 40 ms | 2/3 | 26.32 | 0.102 |

supermpc tracks ~2x tighter. Neither is reliable. Lap-time gap is inside the scatter.

**Hypotheses tested and REFUTED — do not redo these:**

1. *supermpc is 4x slower than bombated.* No. Measured back-to-back under identical load:
   22.97 vs 22.26 ms (3% apart). The earlier 6.76 ms bombated reading was taken on a quiet
   machine. See `times_*.csv`.
2. *Control period too short (25 ms vs ~23 ms solve).* No. At 40 ms mpczinho's lap time was
   identical to 3 d.p. and supermpc got *worse*.
3. *Nonlinear slip constraint is the cost.* No. Removing it: t_lin 13.60 → 13.52 ms.
4. *`num_steps 2→1`, `newton_iter 5→3`.* t_lin 13.45 → 13.25 ms. Reverted.

**50-case weight campaign (`supermpc_full.csv`):** lap time sat at 25.0–26.3 s while `contour`
swung 30→200 (6.7x). Best clean 24.89 s / 0 cones / 0.073 m (`slip0.4`). 21 of 50 abandoned. The
cost weights are not what limits performance.

**Stability sweep (`stability_partial.csv`, 28 of 56 runs before the pause):**
- `rates_2_4` (current): 25.91 s / 0 cones / 0.069 m — best of the partial set
- Heavier rates got monotonically *worse*: `rates_15_30` 27.08 s, `rates_100_200` 32.33 s /
  12 cones. **This contradicts the observation that heavier rate damping helped** — worth a
  second look, since the runs are noisy and n=2.
- Conditioning test: `cond_compressed` (x0.1) 25.39/25.11 s vs `cond_stretched` (x10) 31.31 s +
  1 departure. Weak evidence that badly scaled weights hurt, consistent with a numerical
  component, but not conclusive.
- **The progress:contour ratio cases (`ratio_prog5/10/20/40`, the ones aimed at the braking
  bug) had NOT run yet.** They are cases 20-28 of the sweep.

## Next steps, in order

1. **Re-run the stability sweep from case 20** — the `ratio_prog*` cases directly test the
   braking hypothesis and are the highest-value untested thing:
   `python3 src/control/test/supermpc_sweep.py --stability --repeats 2 --timeout 75
    --max-cones 12 --max-lap-time 50 --max-error 3.0 --out .../stability.csv`
2. If raising progress fixes the braking, re-run the head-to-head against mpczinho.
3. Try the `path_stager` speed-scale lower bound (see above).
4. Only then tune the planner. Tuning it while the controller can still choose to stop is
   pointless.

## Tooling

- `supermpc_sweep.py` — sweeps with early kill on cones, off-track error (>3 m for 1.5 s) and
  lap time. Sweep sets: `--full`, `--stage2`, `--deadline`, `--stability`. Edits YAML by line
  so comments survive; refuses flow-style YAML and verifies key count.
- `capture_failure.py` — records vehicle state, commanded throttle/steering and full trace at
  the moment tracking error diverges. This is what found the braking bug.
- Timing: `scratchpad/timeit.sh` (not preserved — trivial to rewrite; reads
  `/acados/execution_times`, index 1 is `t_tot` in ms).

## Environment caveats

- Foxglove Studio uses ~230–260% CPU; load average reaches 11+ on 16 cores. Absolute solve
  times are inflated. Comparisons are only valid back-to-back in the same session.
- The simulator exits when its stdin hits EOF. Launch with `stdin=DEVNULL` (the harness does).
- `pgrep invictasim_node` does not match — the process name is `invictasim`. Use
  `ps -eo cmd | grep 'lib/invictasim/invictasim'`.
- Never run two simulators at once; they publish to the same topics and silently corrupt data.
