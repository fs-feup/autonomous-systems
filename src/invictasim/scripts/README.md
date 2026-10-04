# InvictaSim scripts

Offline tooling. Nothing here runs as part of the simulator — these are the steps for fitting the
vehicle model to real telemetry and for building new tracks.

Run everything from the **workspace root**.

---

## Fitting the vehicle model to a recording

The vehicle model is identified from recorded car telemetry, not tuned by hand. The chain:

```
   recorded bag
        │
        ├─ (0) control_command_bridge_node.py   only for bags predating ControlCommand
        │
        ├─ (1) rosbag_to_csv.py          ──> tuning_csvs/<name>.csv
        │
        ├─ (2) run_optimizer.sh          ──> performance/invictasim_tuning/
        │          │                          best_parameters.yaml
        │          │                          convergence_log.csv
        │          │                          diagnostic_0.csv
        │          └─ plot_tuning_results.py    did it converge, and where does it diverge
        │
        └─ (3) apply_tuned_parameters.py ──> config/car/**.yaml
```

### 0. Convert a legacy bag

Older bags recorded the controls as `PreviousControlCommand` on `/as_msgs/controls`. Everything
downstream expects `ControlCommand` on `/control/command`:

```sh
python3 src/invictasim/scripts/control_command_bridge_node.py \
    "FSG Autocross DV" control_command_converted
```

Both arguments are positional and both have defaults at the top of the script. The input and output
topics and storage ids are overridable with `--input-topic`, `--output-topic`,
`--input-storage-id` and `--output-storage-id`.

Skip this for any bag already recorded with `/control/command`.

### 1. Export to CSV

```sh
python3 src/invictasim/scripts/rosbag_to_csv.py <bag> --output <name>.csv
```

Reads `/control/command`, `/state_estimation/{vehicle_pose,velocities}` and the four wheel plus
motor rpm topics, synchronises them on true event time, and always writes into
`src/invictasim/tuning_csvs/`. Both arguments are prompted for if omitted.

The `csvs:` list in the tuning config names files in that folder, so `<name>.csv` has to match what
step 2 expects to read.

### 2. Run the optimizer

The optimizer is a separate C++ target, off by default:

```sh
colcon build --packages-select invictasim --cmake-args -DBUILD_VEHICLE_MODEL_OPTIMIZER=ON
./src/invictasim/scripts/run_optimizer.sh config/invictasim/tuning/02_feup_trackdrive_tuning.yaml
```

It replays the CSV through `inline_02.hpp` — the same physics as the live simulator, with no ROS in
the loop — and searches with either a genetic algorithm or simulated annealing, whichever the config
selects. Which parameters it searches, and their bounds, come from the same config.

The optimizer also takes `--set name=value` to pin a parameter and `--diagnose` to replay once and
print the per-signal errors, which is how you check a single parameter by hand without starting a
search.

**After a run**, look at what happened:

```sh
python3 src/invictasim/scripts/plot_tuning_results.py
```

Writes `convergence.png` (score and RMSEs against both time and evaluation count) and
`trajectory.png` (simulated against real XY path, plus per-signal time series) from the artefacts in
`performance/invictasim_tuning/`.

### 3. Apply the result

```sh
python3 src/invictasim/scripts/apply_tuned_parameters.py \
    performance/invictasim_tuning/best_parameters.yaml config/car 02
```

Arguments are the result file, the car config directory, and the car name (defaults to `02`).

The optimizer reports parameters under its own aliases (`tire.lateral_peak_scale` and so on). This
maps them back to the real keys (`LMUY`) and the file each one lives in, across `config/car/`.
Doing it by hand means editing several YAML files and getting every alias right.

---

## Identifying the steering motor

`steering_motor_model: "first_order"` has a single time constant. This fits it from a recording that
has both the commanded and the measured steering angle:

```sh
python3 src/invictasim/scripts/fit_steering_motor.py <bag> [<bag> ...]
```

Reads `/control/command` against `/vehicle/steering_motor_state`, sweeps the time constant between
`--tau-min` and `--tau-max`, and reports the best fit to put in
`config/car/steering_motor_model/`. Accepts several bags at once, and `--start-s`, `--duration-s`
and `--min-abs-steering` to restrict the fit to a useful window.

---

## Building a track from a recording

```sh
# From a recording
python3 src/invictasim/scripts/generate_track_from_slam.py --bag <bag> --track-name <name>

# Or live, subscribing to a running SLAM node
python3 src/invictasim/scripts/generate_track_from_slam.py --track-name <name>
```

Takes the SLAM map, vehicle pose and lap counter either out of a bag or off live topics, and writes
a track YAML into `resources/tracks/`, which `track_name` can then select. This is how a real
circuit the car has driven becomes a simulated one.

Check the result before using it: a noisy map gives noisy cone positions, and the timing line is
placed from the recorded start pose.
