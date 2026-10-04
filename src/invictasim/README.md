# InvictaSim

InvictaSim is the FS FEUP simulator. It stands in for the car so the autonomous pipeline —
perception, state estimation, planning and control — can be developed and tested without the real
vehicle.

It is a single ROS 2 node that integrates a physical vehicle model at 1 kHz, simulates the car's
sensors, and publishes everything the rest of the stack expects to read from the car. It can also
stand in for state estimation and perception, so you can bring up just the part of the pipeline you
are working on.

```
       ┌──────────────── invictasim node ────────────────┐
       │                                                 │
 ┌─────┴─────┐      ┌──────────────┐      ┌──────────────┴──┐
 │   input   │─────>│  simulation  │─────>│     output      │
 │  adapter  │      │     loop     │      │    adapter      │
 └─────┬─────┘      └──────┬───────┘      └──────┬──────────┘
       │                   │                     │
  /control/command    vehicle model          ~34 ROS topics
  /path_planning/path  + sensors             + TF + RViz markers
       ▲               + statistics                │
       │                                           │
       └──────── control ◀── planning ◀────────────┘
```

---

## Contents

- [How it runs](#how-it-runs) — threads, the simulation loop, snapshots
- [The vehicle model](#the-vehicle-model) — state vector, integration, sub-models
- [Simulated sensors](#simulated-sensors) — IMU, wheel speeds, LiDAR
- [Standing in for the pipeline](#standing-in-for-the-pipeline) — the `use_simulated_*` flags
- [Topics](#topics) — everything published, and what controls its rate
- [Tracks](#tracks) — layout format, start position, timing lines
- [Statistics](#statistics) — lap timing, cone penalties, tracking error
- [Visualization](#visualization) — RViz markers and the generated track surface
- [Configuration](#configuration) — every file, and how they are read
- [Build and run](#build-and-run)
- [Driving it by hand](#driving-it-by-hand) — keyboard mode and hotkeys
- [Tuning the vehicle model](#tuning-the-vehicle-model) — the offline optimizer
- [Tests](#tests)
- [Package layout](#package-layout)

---

## How it runs

`main.cpp` builds three objects and three threads.

| Thread | What it does |
|---|---|
| Simulation | Runs `InvictaSim::run()`, a tight loop calling `simulation_step()` |
| ROS executor | Spins the input and output adapter nodes, so every ROS callback and publish timer runs here |
| Terminal listener | Reads hotkeys from `/dev/tty`, started by the input adapter base class |

The adapters are chosen at startup from `input_adapter` and `output_adapter` in the config, through
the registries in [include/io/input/map.hpp](include/io/input/map.hpp) and
[include/io/output/map.hpp](include/io/output/map.hpp). The ROS adapters are themselves
`rclcpp::Node`s, so `main` hands them to the executor; a non-ROS adapter (the keyboard one) gets its
own thread instead.

### The simulation step

Each pass of [`simulation_step()`](src/simulator/invictasim.cpp):

1. **Applies a pending reset.** The `r` hotkey only sets a flag; the reset itself happens here, on
   the simulation thread, so it cannot race the step that is reading the state.
2. **Sleeps until the next step.** The period is `1 / sim_frequency` divided by `sim_speed`.
3. **Measures real elapsed time** and scales it by `sim_speed` to get `sim_dt`. `sim_dt` is
   **capped at 10 step periods**: if the thread stalls, the simulation briefly falls behind real
   time rather than asking the vehicle model to integrate further than it stably can. Overruns are
   reported, aggregated, every 5 seconds.
4. **Steps the vehicle model** with the latest control input.
5. **Updates the statistics** with the new state and the latest planning path.
6. **Builds the output snapshots** and swaps them in under a mutex.

### Snapshots

The simulation thread never publishes, and the ROS thread never touches the vehicle model. They
meet at six plain structs in [include/io/output/output_snapshot.hpp](include/io/output/output_snapshot.hpp):

| Snapshot | Holds |
|---|---|
| `VehicleModelSnapshot` | Full vehicle state: tyre forces, slips, motor, battery, aero, pose |
| `SensorsSnapshot` | What the simulated sensors report, with their errors applied |
| `VehicleStateSnapshot` | The simulated state estimator's output, with covariances |
| `MapSnapshot` | Ground-truth cones, the simulated SLAM map, perception output |
| `StatisticsSnapshot` | Lap times, cones hit, tracking error |
| `ExecutionTimesSnapshot` | Per-subsystem timing of the last step |

The simulation thread writes all six at the end of a step while holding `output_snapshot_mutex_`;
a publisher takes a copy of the one it needs. Keeping the lock to a struct copy is what lets the
model run at 1 kHz while publishers tick at their own rates.

The control input travels the other way under `input_mutex_`. Throttle and steering are copied by
value, but the cone map and planning path are held as `shared_ptr<const vector<...>>` and swapped
wholesale by the setters, so a step does not deep-copy a thousand cones a thousand times a second.

### Publish rates

The output adapter does not publish from the simulation loop. At startup it reads
`publish_frequencies` from [config/invictasim/output/ros.yaml](../../config/invictasim/output/ros.yaml),
groups every publisher by its requested frequency, and creates **one wall timer per distinct
frequency**. Each tick refreshes the snapshots that group needs and calls its publishers.

A frequency of `0` means the publisher is never registered and its timer never exists, so disabling
a topic costs nothing at runtime.

---

## The vehicle model

Two cars are available, selected by `vehicle_model`: `"02"` and `"03"`. Both are rear-wheel-drive
four-wheel models with load transfer and a Pacejka tyre, assembled from `motion_lib` sub-models.

### State vector

`FSFEUP02Model` integrates 13 states ([include/vehicle_model/FSFEUP02.hpp](include/vehicle_model/FSFEUP02.hpp)):

```
VX, VY        body-frame velocities              PX, PY     map-frame position
YAW_RATE      yaw rate                           ST_ANGLE   steering rack angle
YAW           heading                            FL_W … RR_W per-wheel angular speed
AX, AY        low-pass filtered specific force
```

`AX`/`AY` are states rather than outputs because they drive the load transfer model, and the chassis
takes time to settle onto its springs — they are filtered with a 0.1 s time constant. The
**unfiltered** specific force is what gets published as the car's acceleration, so consumers do not
inherit that lag.

### Integration

`step()` does not integrate the incoming `dt` in one go. The driven-wheel/tyre-slip mode is stiff —
its time constant is roughly `I · denom / (kxk · r²)`, under a millisecond at low speed — and
explicit RK4 is only stable below that. Since the stable step grows with speed, `step()` splits `dt`
into substeps: one at cruise, up to 64 at a standstill.

Each substep:

- computes the powertrain torque once, advancing the battery, motor and inverter state;
- evaluates the state derivative four times for RK4, then a fifth time at the integrated state with
  `write_telemetry = true`, which is what fills in tyre forces, slips and loads for publishing;
- applies the post-integration constraints: wheel lock-up under braking, EBS, and a dead-stop clamp
  that stops the car oscillating around zero.

Lock-up and EBS are **constraints on the integrated state, not derivatives**. A "teleport to zero"
derivative inside RK4 only decays the wheel by a fixed fraction per substep instead of stopping it.

The offline tuning model, [inline_02.hpp](src/vehicle_model/optimizer/inline_models/inline_02.hpp),
substeps identically, so parameters tuned offline hold in the live simulation. **If you change the
plant, change it there too.**

### Sub-models

Each line of [config/invictasim/vehicle_models/02.yaml](../../config/invictasim/vehicle_models/02.yaml)
picks an implementation and a parameter set. The implementation names come from the registries in
`motion_lib`; the parameter sets name files under `config/car/`.

| Sub-model | Registered options |
|---|---|
| `tire_model` | `pacejka_MF6_2`, `pacejka_combined_slip` |
| `motor_model` | `map_based_motor` |
| `battery_model` | `thevenin` |
| `transmission_model` | `diff_salisbury` |
| `inverter_model` | `ramp` |
| `brake_model` | `default` |
| `aero_model` | `default_aero` |
| `load_transfer_model` | `vd_model`, `rigid_body` |
| `steering_model` | `ackerman_steering`, `parallel_front_steering` |
| `steering_motor_model` | `first_order`, `pid` |

All ten are validated at startup; an unknown name fails with the list of what is accepted.

### Braking: manual versus autonomous

`control_mode` decides what a **negative throttle** means, and the distinction is deliberate:

- **`manual`** — the driver has a brake pedal. A negative command builds friction brake torques and
  the inverter is cut.
- **`autonomous`** — the car has no brake actuator. A negative command is a **regen request only**,
  and goes to the inverter, which limits it to `regen_torque_fraction` of its torque limit. The
  friction brakes are never engaged.

This is why the acados solvers taper their regen request with speed: regen reaches the inverter as
negative motor torque, and `RampedInverter` slews towards the request rather than applying it at
once, so regen asked for at speed is still arriving once the car has stopped.

---

## Simulated sensors

The simulator does not publish ground truth on the sensor topics. Each channel goes through an
error model in [src/sensors/](src/sensors/), configured under
[config/invictasim/sensors/](../../config/invictasim/sensors/).

### IMU

`measured = true · (1 + scale_factor) + bias + N(0, noise_std_dev)`

Applied to longitudinal and lateral acceleration and to yaw rate. The scale factor error is
proportional to the **signed** value, so it keeps the sign of what it distorts.

### Wheel speed sensors

Per wheel, in order: a **speed cutoff**, then noise, outliers, dropout and ADC quantisation.

The cutoff is evaluated on the **true** wheel speed, not the noisy reading — below it no teeth pass
the pickup, so a parked car reports exactly zero rather than noise about zero. `quantization_bits`
is an ADC resolution over a ±4000 rpm full scale; `0` disables it.

All four wheels are published, each on its own topic, with only its own field set.

### LiDAR cone pipeline

`SimulatedPerception` turns the ground-truth map into what the sensor would see, in the **vehicle
frame**:

1. Transform each cone into the vehicle frame.
2. **Field of view.** Horizontal is a simple azimuth test. Vertical uses the elevation of the cone's
   *centroid* as seen from the sensor, in the sensor's own frame, so the mounting pitch brings near
   cones into view rather than shifting where they are reported. This is what `height` and
   `cone_height` are for: the geometry sets a near blind radius, currently about 2.4 m.
3. **Detection probability**, a sigmoid through 50 % at `max_range`.
4. **Position noise**, optionally growing with range.
5. **Spurious detections**, at a rate per *second* rather than per call, so the result does not
   depend on the simulation step. A fraction of them are retained across frames, capped at 32.

Only constructed when `use_simulated_perception` is set.

---

## Standing in for the pipeline

Three flags in [config/global/global_config.yaml](../../config/global/global_config.yaml) decide how
much of the stack the simulator replaces. They let you run just the node you are working on.

| Flag | `true` | `false` |
|---|---|---|
| `use_simulated_se` | Simulator publishes a perfect map, pose and lap counter | Simulator *subscribes* to `/state_estimation/map`; run SLAM |
| `use_simulated_perception` | Simulator runs the LiDAR model and publishes cones | Simulator subscribes to `/perception/cones`; run perception |
| `use_simulated_velocities` | Simulator publishes velocities with zero covariance | Run velocity estimation |

The simulated state estimator is **perfect**: it reports the model's own state with zero covariance.
The simulated SLAM map is the ground truth in the map frame, representing a completed mapping run.
Perception, in contrast, reports only what the sensor can currently see.

### How the stack connects to it

The simulator publishes under an `invictasim/` prefix. The other nodes reach it through their own
**adapters**, selected by `adapter: "invictasim"` in the global config:

- [`src/control/src/adapter/invictasim.cpp`](../control/src/adapter/invictasim.cpp)
- [`src/planning/src/adapter_planning/invictasim.cpp`](../planning/src/adapter_planning/invictasim.cpp)

There are no launch-file remappings; the adapters subscribe to the prefixed names directly. A full
loop is `ros2 launch launcher sim_control_planning.launch.py`:

```
invictasim ──/invictasim/state_estimation/{map,vehicle_pose,lap_counter}──> planning
invictasim ──/invictasim/operational_status (go signal + mission)─────────> planning, control
invictasim ──/invictasim/state_estimation/velocities──────────────────────> control
invictasim ──/invictasim/vehicle_model/status (accel, steering, wheels)───> control
planning   ──/path_planning/path──────────────────────────────────────────> control, invictasim
control    ──/control/command─────────────────────────────────────────────> invictasim
```

`operational_status` carries the go signal and the mission, and **planning and control both gate on
it**, so its rate sets how quickly the pipeline starts.

### Which combinations work

Verified by running the real nodes, with `decoupled` + `mpczinho` driving:

| `se` | `perception` | `velocities` | Nodes you also run | Result |
|---|---|---|---|---|
| `true` | `true` | `true` | — | Car drives. Simulator replaces perception, SLAM and velocity estimation. |
| `false` | `true` | `true` | `slam` | Car drives. Simulated LiDAR feeds the real SLAM, which maps the track and localises. |
| `true` | `true` | `false` | `velocity_estimation` | Car drives. The simulated IMU, wheel speed and resolver channels feed the real estimator. |
| `true` | `false` | `true` | — | Car drives on the simulated SLAM map. The simulator's `/perception/cones` subscription simply stays empty. |
| `false` | `false` | `true` | `slam` | Car sits still, no errors. Nothing can provide the map — see below. |

The last row is the real boundary: **the simulator cannot drive the real perception node.** Perception
consumes LiDAR point clouds (`/hesai/pandar`, `/rslidar_points/pre_processed`) and the simulator does
not generate them — it simulates perception's *output* (cones with field of view, range, detection
probability, noise and spurious detections), not its input. To exercise the real perception node you
need a bag with LiDAR data or the sensor itself.

Everything else composes freely. `velocity_estimation` can be combined with real SLAM as well, since
they read different simulator topics.

---

## Topics

Every rate below is the key under `publish_frequencies` in
[config/invictasim/output/ros.yaml](../../config/invictasim/output/ros.yaml). All types are
`custom_interfaces/msg` unless noted.

### Vehicle model telemetry

| Topic | Type | Rate key |
|---|---|---|
| `invictasim/vehicle_model/tire/forces` | `TireForces` | `tire` |
| `invictasim/vehicle_model/tire/slip_ratio` | `WheelScalars` | `tire` |
| `invictasim/vehicle_model/tire/slip_angle` | `WheelScalars` | `tire` |
| `invictasim/vehicle_model/motor` | `MotorState` | `motor` |
| `invictasim/vehicle_model/battery` | `BatteryState` | `battery` |
| `invictasim/vehicle_model/transmission` | `WheelScalars` | `transmission` |
| `invictasim/vehicle_model/aero` | `AeroForces` | `aero` |
| `invictasim/vehicle_model/status` | `VehicleStateVector` | `status` |
| `invictasim/input` | `ControlCommand` | `status` |
| `invictasim/execution_times` | `ExecutionTimes` | `execution_time` |

`vehicle_model/status` is the controller's input: pose, velocities, accelerations, steering angle and
wheel speeds in rpm.

### Sensors

| Topic | Type | Rate key |
|---|---|---|
| `invictasim/imu/free_acceleration` | `geometry_msgs/Vector3Stamped` | `imu` |
| `invictasim/imu/angular_velocity` | `geometry_msgs/Vector3Stamped` | `imu` |
| `invictasim/wss/{front_left,front_right,rear_left,rear_right}` | `WheelRPM` | `wheel_speed` |
| `invictasim/resolver` | `std_msgs/Float64` | `resolver` |
| `invictasim/steering_angle_sensor` | `SteeringAngle` | `steering` |

### Maps and simulated state estimation

| Topic | Type | Rate key | Condition |
|---|---|---|---|
| `invictasim/map` | `ConeArray` | `ground_truth` | always |
| `invictasim/state_estimation/map` | `ConeArray` | `simulated_slam` | `use_simulated_se` |
| `invictasim/state_estimation/vehicle_pose` | `Pose` | `pose` | `use_simulated_se` |
| `invictasim/state_estimation/vehicle_state_vector` | `VehicleStateVector` | `state_vector` | `use_simulated_se` |
| `invictasim/state_estimation/lap_counter` | `std_msgs/Float64` | `lap_counter` | `use_simulated_se` |
| `invictasim/state_estimation/velocities` | `Velocities` | `velocities` | `use_simulated_velocities` |
| `invictasim/perception/cones` | `PerceptionOutput` | `perception` | `use_simulated_perception` |
| `invictasim/operational_status` | `OperationalStatus` | `operational_status` | always |

### Statistics

| Topic | Type | Rate key |
|---|---|---|
| `invictasim/statistics/lap_summary` | `LapSummary` | `lap_summary` |
| `invictasim/statistics/lap_current` | `LapCurrent` | `lap_current` |
| `invictasim/statistics/control_statistics` | `ControlStatistics` | `control_statistics` |

`lap_summary` is `transient_local` and carries the **whole** lap history each time, so a subscriber
that joins late still gets the full table.

### Visualization

| Topic | Rate key |
|---|---|
| `invictasim/visualization/vehicle` | `car` |
| `invictasim/visualization/ground` | `ground` |
| `invictasim/visualization/ground_truth_cones` | `ground_truth_cones` |
| `invictasim/visualization/slam_cones` | `slam_cones` |
| `invictasim/visualization/perception_cones` | `perception_cones` |

All `visualization_msgs/MarkerArray`.

### Subscriptions and frames

Set in [config/invictasim/input/ros.yaml](../../config/invictasim/input/ros.yaml):

| Topic | Purpose |
|---|---|
| `/control/command` | Per-wheel throttle and steering; this is what drives the car |
| `/path_planning/path` | Used for tracking-error statistics and path visualization |
| `/state_estimation/map` | Only when `use_simulated_se` is `false` |
| `/perception/cones` | Only when `use_simulated_perception` is `false` |

TF: `map` → `car`. Map-frame data uses `map`, vehicle-frame data uses `base_link`, and car-attached
markers use `car`.

---

## Tracks

A track is a YAML file in [resources/tracks/](resources/tracks/). `track_name` is **validated
against that directory** at startup, so a typo fails with the list of installed tracks rather than a
cryptic file error.

```yaml
track:
  start_position: [-16.68, 0]        # where the car starts, map frame
  timing_lines:                      # one or more [[x1,y1],[x2,y2]] segments
  - [[0, -3], [0, 3]]
  cones:
  - [-15, 1.65, "orange_cone"]       # [x, y, colour]
```

- **Colours**: `blue_cone`, `yellow_cone`, `orange_cone`, `large_orange_cone`.
- **`timing_line`** (singular) is accepted for a single line. With neither key, a 10 m line is
  synthesised across the start position.
- **Duplicate cones** of the same colour at the same place are dropped on load.
- Every timing line is **fitted to the local track width** using the nearest boundary cones, so the
  gate spans the track rather than whatever the YAML literally said.
- **`start_gap_to_timing_line`** (optional) places the car's *foremost point* that far behind the
  first timing line, computed from the loaded car's hitboxes. `Acceleration.yaml` uses it so a
  standing start is timed identically for the 02 and the 03. Without it, `start_position` is used
  as given.

The car always starts at **yaw 0**, facing +X; there is no start-heading field.

---

## Statistics

[src/statistics/](src/statistics/) runs three things every step, and only once a lap has started.

**Lap timing** ([lap_timer_statistics.cpp](src/statistics/lap_timer_statistics.cpp)) tracks the
car's *nose* — the foremost hitbox point — across the timing line, using a signed distance plus a
gate test so passing outside the line's span does not count. A lap needs both a minimum elapsed time
and a minimum driven distance, so a car that stops on the line and rolls back over it does not
score one. `acceleration` is special-cased to use two separate gates: a start line and the 75 m
line.

**Cone collisions** ([collision_statistics.cpp](src/statistics/collision_statistics.cpp)) test the
cone as a circle against the car's hitbox rectangles in the car frame. A distance pre-filter rejects
almost everything first, since this runs at 1 kHz against the whole cone list. Each cone counts once
per lap, and the penalty per cone comes from the discipline.

**Tracking error** ([tracking_statistics.cpp](src/statistics/tracking_statistics.cpp)) compares the
car against the latest planning path for cross-track and velocity error.

Each completed lap is also appended to a CSV, written to `$INVICTASIM_RUNS_DIR` or
`~/.local/share/invictasim/runs`. Failures there are warnings, never exceptions — lap traces are a
convenience and must not take the simulation down.

---

## Visualization

Add the `invictasim/visualization/*` topics in RViz with the fixed frame set to `map`.

The **car** is built from `.glb` meshes under [resources/meshes/car/](resources/meshes/car/), chosen
by the car config, with offsets from that folder's `config.yaml`. Wheels spin at their simulated
speed, and the steering wheel turns by `steering_rotation_multiplier`. Hitboxes can be drawn by
setting `hitboxes.visualize`.

The **track surface** has two modes, set by `track_visualization` in
[resources/meshes/ground/config.yaml](resources/meshes/ground/config.yaml):

- `generated_track` — builds asphalt and red/white curb ribbons from the cone positions. Handles the
  three layouts separately: a closed loop joins each colour end to end, acceleration keeps the
  boundaries as open lanes, and skidpad splits each colour into its two circles by looking for jumps
  in cone spacing.
- `asphalt_texture_everywhere` — a flat textured plane.

The grass plane is **generated at runtime** so its texture tiles per metre instead of stretching with
the plane. It is written into the package's share directory (which is what `package://` resolves to),
atomically via a temporary file and a rename, and falls back to the shipped fixed-scale plane if the
install is read-only.

---

## Configuration

Everything is read through `common_lib::config_load`, which resolves names against `config/`.

| File | Contents |
|---|---|
| [config/invictasim/global.yaml](../../config/invictasim/global.yaml) | Adapters, car, control mode, frequency, speed, track |
| [config/global/global_config.yaml](../../config/global/global_config.yaml) | Discipline and the `use_simulated_*` flags, shared with the rest of the stack |
| [config/invictasim/vehicle_models/{02,03}.yaml](../../config/invictasim/vehicle_models/) | Which sub-model and parameter set each part of the car uses |
| [config/invictasim/input/ros.yaml](../../config/invictasim/input/ros.yaml) | Subscription topics and queue depth |
| [config/invictasim/input/keyboard.yaml](../../config/invictasim/input/keyboard.yaml) | Throttle and steering rates, SDL window geometry |
| [config/invictasim/output/ros.yaml](../../config/invictasim/output/ros.yaml) | `publish_frequencies` per topic group |
| [config/invictasim/sensors/](../../config/invictasim/sensors/) | IMU, WSS and LiDAR error models |
| [config/invictasim/tuning/](../../config/invictasim/tuning/) | Offline optimizer settings |
| [resources/meshes/*/config.yaml](resources/meshes/) | Mesh offsets, hitboxes, track surface, lap rules |

Adapter and sensor configs are read **key by key**: anything omitted keeps the default compiled
into the class, so a config only needs the values actually being overridden. The simulator-wide
config in `global.yaml` is the opposite — its keys are **required**, and a missing one fails naming
the file and the key.

`discipline` lives in the global config and selects the lap timing rules. A discipline that does not
suit the track is warned about at startup, because lap times would otherwise be quietly meaningless.

Code: [src/config/config.cpp](src/config/config.cpp) loads and validates,
[src/config/validate.cpp](src/config/validate.cpp) holds the checks and their messages,
[include/config/yaml_helpers.hpp](include/config/yaml_helpers.hpp) is the optional-key helper, and
[include/config/resource_paths.hpp](include/config/resource_paths.hpp) resolves `resources/` through
the ament index so an installed package behaves like a source build.

---

## Build and run

```sh
colcon build --packages-select invictasim
source install/setup.bash
```

Just the simulator:

```sh
ros2 launch invictasim invictasim.launch.py   # or: ros2 run invictasim invictasim
```

With planning and control:

```sh
ros2 launch launcher sim_control_planning.launch.py
```

The launch file shuts the whole description down when the simulator exits, so closing the simulator
closes the run.

---

## Driving it by hand

Set `input_adapter: "keyboard"`. The adapter opens an SDL window showing throttle and steering
bars, and `S` sends a negative throttle — so also set `control_mode: "manual"` if you want the
friction brakes, otherwise `S` only gives you regen (see
[braking](#braking-manual-versus-autonomous)).

| Key | Action |
|---|---|
| `W` / `S` | Throttle up / brake |
| `A` / `D` | Steer left / right |
| Close window | Stop keyboard input |

Throttle and steering ramp while a key is held, by `throttle_step` and `steering_step` per loop
iteration, and recentre when released.

**The SDL window needs a display.** Without one, `SDL_Init` fails, the adapter reports it and
returns, and the simulator runs with no input source at all.

### Hotkeys

These work in the launching terminal in any mode, from the terminal listener thread:

| Key | Action |
|---|---|
| `SPACE` | Raise the go signal |
| `+` / `-` | Change `sim_speed` by 0.25, clamped to [0.25, 10] |
| `e` | Toggle the EBS |
| `r` | Reset: car back on the start line, state and statistics cleared |

Under `ros2 launch` the output is a pipe rather than a terminal; the listener detects that, says so,
and disables the hotkeys rather than corrupting the terminal.

---

## Tuning the vehicle model

The vehicle model is fitted offline against real telemetry, not by hand.

Run all of these from the workspace root.

```sh
# 1. Export a rosbag to CSV. The output always lands in src/invictasim/tuning_csvs/.
#    Both arguments are prompted for if omitted.
python3 src/invictasim/scripts/rosbag_to_csv.py <bag> --output <name>.csv

# 2. Build the optimizer (off by default)
colcon build --packages-select invictasim --cmake-args -DBUILD_VEHICLE_MODEL_OPTIMIZER=ON

# 3. Run it, pointing at a tuning config
./src/invictasim/scripts/run_optimizer.sh config/invictasim/tuning/02_feup_trackdrive_tuning.yaml
```

The tuning config's `csvs:` list names files inside `tuning_csvs/`, so step 1's output filename has
to match what step 3 expects to read.

The optimizer replays the CSV through `inline_02.hpp` — the same physics as the live simulator, with
no ROS in the loop — scores the trajectory against the recording, and searches with either a genetic
algorithm or simulated annealing. The tuning config lists which parameters to search and their
bounds.

Telemetry CSVs are **not tracked in git** (see `tuning_csvs/.gitignore`); export your own.

`apply_tuned_parameters.py` then writes the result back into the car configs, mapping the
optimizer's parameter aliases to the real keys, and `plot_tuning_results.py` shows whether the run
converged and where the simulation diverges from the recording.

The rest of [scripts/](scripts/) covers the other offline jobs: converting a legacy bag, fitting the
steering motor time constant, and building a track YAML from a recorded SLAM map. Each step, with
its real arguments, is in [scripts/README.md](scripts/README.md).

Setting `rosbag_evaluator: true` enables `TuningEvaluator`, which does the same comparison **live**
against a replaying bag and publishes the running errors under `/invictasim/tuning/*`.

---

## Tests

```sh
colcon test --packages-select invictasim
colcon test-result --all --verbose
```

Covers track loading for every shipped track, the inverter's slew-rate behaviour, the wheel speed
sensor, and vehicle model behaviour — that it pulls away, steers symmetrically, brakes according to
the control mode, stops under EBS, resets repeatably, and never goes non-finite over a long run.

---

## Package layout

```text
src/invictasim/
├── include/ , src/            # Mirrored headers and sources
│   ├── config/                # Loading, validation, resource paths
│   ├── io/input/              # ROS and keyboard adapters, terminal hotkeys
│   ├── io/output/             # ROS publishers, visualization, snapshots
│   ├── sensors/               # IMU, wheel speed, LiDAR error models
│   ├── simulator/             # The simulation loop
│   ├── statistics/            # Lap timing, collisions, tracking error
│   ├── track/                 # Track loading
│   ├── tuning/                # Live rosbag evaluator
│   └── vehicle_model/         # FSFEUP02, FSFEUP03
│       └── optimizer/         # Offline parameter optimizer and inline_02
├── launch/
├── resources/
│   ├── meshes/                # Car, cone and ground meshes plus their config
│   └── tracks/                # Track layouts
├── scripts/                   # Offline tooling; see scripts/README.md
├── test/
└── tuning_csvs/               # Telemetry exports (gitignored)
```
