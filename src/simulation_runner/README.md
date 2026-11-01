# simulation_runner

Runs the InvictaSim full stack through a set of scenarios and checks the driving performance
against each scenario's criteria. Used by the `simulate` job in `.github/workflows/build.yml` to
gate pull requests, and usable by hand for the same check.

Each scenario is one closed-loop run: simulated sensors feed the velocity estimator, that and
simulated perception feed SLAM, and planning and control drive off SLAM's map and pose.

## Running it

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run simulation_runner run                       # every scenario
ros2 run simulation_runner run --only trackdrive     # one of them
ros2 run simulation_runner run --keep-going          # do not stop at the first failure
ros2 run simulation_runner run --list                # show what is defined
```

Exits non-zero when a required scenario fails. Logs, `results.json` and `summary.md` are written
to `simulation_results/` (`--output`), and the summary is appended to `$GITHUB_STEP_SUMMARY`.

Runs on `ROS_DOMAIN_ID=42` so rviz, Foxglove and `ros2 topic` see it; `--ros-domain-id N` changes
that. Leftover nodes from an earlier run are killed before and after every run, since on a shared
domain they would otherwise join the graph being measured.

## Discord

After the scenarios finish the outcome is posted to the webhook in `$DISCORD_WEBHOOK_URL`, if set.
`--author` is looked up in [`config/simulation/discord_users.yaml`](../../config/simulation/discord_users.yaml)
to tag someone; unlisted authors are named but not tagged. `--discord-dry-run` prints the message
instead of sending it.

## Defining scenarios

Scenarios live in [`config/simulation/scenarios.yaml`](../../config/simulation/scenarios.yaml),
which documents every field. Adding a discipline, track or criterion is a change to that file.

## What the runner changes, and what it does not

A run needs the stack pointed at the simulator, so the runner temporarily rewrites:

| File | Keys |
| --- | --- |
| `config/global/global_config.yaml` | `adapter`, `discipline`, `use_simulated_se`, `use_simulated_velocities`, `use_simulated_perception`, `use_simulated_planning` |
| `config/invictasim/global.yaml` | `track_name`, `control_mode`, `input_adapter`, `output_adapter`, `initial_go_signal` |
| `config/slam/invictasim.yaml` | `using_preloaded_map` |

Everything else is left as committed: control gains, planning tuning, SLAM and velocity
estimation parameters and sensor noise all belong to the pull request. The rewrites preserve
comments, revert when the run ends, and are restored on the next start if a run was killed.

Settings that make a run impossible are reported before anything is launched.

## How a run ends

A ROS node subscribes to `/invictasim/statistics/lap_summary` (laps, cone hits, tracking errors)
and `/invictasim/vehicle_model/status` (speed), and ends the run once `min_laps` are complete
and, with `wait_for_stop`, the car is below `stop_speed`. `timeout` is the wall-clock backstop.

The simulator knows nothing about scenarios, criteria or when a run should stop.
