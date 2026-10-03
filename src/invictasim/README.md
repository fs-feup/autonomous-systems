# Invictasim

Invictasim is the Formula Student FEUP simulator package for autonomous system development. It
supports ROS-driven input and SDL keyboard input, publishes vehicle/track/visualization outputs,
and runs with configurable vehicle and track models.

## Current Package Structure

```text
src/invictasim/
├── include/
│   ├── config/            # Config loading, validation and resource path resolution
│   ├── io/
│   │   ├── input/         # Control command sources: ROS topics or an SDL keyboard window
│   │   └── output/        # ROS publishers, visualization and the output snapshots
│   ├── sensors/           # Simulated IMU, wheel speed sensors and LiDAR cone pipeline
│   ├── simulator/         # Simulation loop
│   ├── statistics/        # Lap timing, cone collisions and tracking error
│   ├── track/             # Track loading from resources/tracks
│   ├── tuning/            # Rosbag replay evaluator
│   └── vehicle_model/     # Vehicle models (02, 03) over motion_lib sub-models
├── launch/
│   └── invictasim.launch.py
├── resources/
│   ├── meshes/            # Car, cone and ground meshes, plus their configuration
│   └── tracks/            # Track layouts; track_name is validated against this directory
├── scripts/               # Offline tooling: rosbag to CSV, parameter tuning, plotting
├── src/                   # Mirrors include/, plus main.cpp
│   └── vehicle_model/
│       └── optimizer/     # Standalone offline parameter optimizer (BUILD_VEHICLE_MODEL_OPTIMIZER)
├── test/
├── tuning_csvs/           # Telemetry exports for the optimizer (not tracked in git)
├── CMakeLists.txt
└── package.xml
```

Lap traces are written to `$INVICTASIM_RUNS_DIR`, or `~/.local/share/invictasim/runs` when that is
unset.

## Runtime Overview

- `invictasim` executable starts simulator, input adapter, and output adapter.
- Input adapter options:
  - `ros`: subscribes to `/control/command`
  - `keyboard`: opens an SDL window (`W/S` throttle, `A/D` steering)
- Output adapter currently available:
  - `ros`: publishes simulator telemetry, input echo, track, TF, and visualization topics.

Adapter mapping is defined in:

- [include/io/input/map.hpp](include/io/input/map.hpp)
- [include/io/output/map.hpp](include/io/output/map.hpp)

## Configuration Files Used

Invictasim reads configuration through `common_lib::config_load`:

- [config/invictasim/global.yaml](../../config/invictasim/global.yaml)
  - `input_adapter`, `output_adapter`, `vehicle_model`, `sim_frequency`, `track_name`
- [config/global/global_config.yaml](../../config/global/global_config.yaml)
  - global discipline/system parameters used by the simulator context
- [config/invictasim/vehicle_models/02.yaml](../../config/invictasim/vehicle_models/02.yaml)
  - which sub-model implementation and parameter set each part of the car uses, named after the
    `vehicle_model` value ("02", "03")

Each adapter owns its own config file, named after the value of `input_adapter` / `output_adapter`
and loaded by the adapter itself:

- [config/invictasim/input/ros.yaml](../../config/invictasim/input/ros.yaml)
  - subscription topics and queue depth
- [config/invictasim/input/keyboard.yaml](../../config/invictasim/input/keyboard.yaml)
  - throttle/steering rates and SDL window geometry
- [config/invictasim/output/ros.yaml](../../config/invictasim/output/ros.yaml)
  - `publish_frequencies` per topic group; a frequency of `0` disables that publisher entirely

The simulated sensors read their own error models, independently of the adapters:

- [config/invictasim/sensors/imu.yaml](../../config/invictasim/sensors/imu.yaml)
  - accelerometer and gyroscope bias, scale factor and noise
- [config/invictasim/sensors/wss.yaml](../../config/invictasim/sensors/wss.yaml)
  - per-wheel noise, outliers, dropout, speed cutoff and ADC resolution
- [config/invictasim/sensors/perception.yaml](../../config/invictasim/sensors/perception.yaml)
  - LiDAR geometry, field of view, detection probability and spurious detections; only loaded
    when `use_simulated_perception` is set

These files are read key by key: anything left out keeps the default compiled into the adapter, so
a config file only needs the values actually being overridden.

`track_name`, the adapters and every sub-model name are validated on startup. An unknown value
fails with a message listing what is accepted, and a discipline that does not suit the track is
warned about, because lap timing follows the discipline.

Code reference for config loading:

- [src/config/config.cpp](src/config/config.cpp) — simulator-wide parameters
- [src/config/validate.cpp](src/config/validate.cpp) — startup validation and its messages
- [include/config/yaml_helpers.hpp](include/config/yaml_helpers.hpp) — optional-key helper used by
  the adapters
- [include/config/resource_paths.hpp](include/config/resource_paths.hpp) — resolves `resources/`
  through the ament index, so an installed package works the same as a source build

## Build

From workspace root:

```sh
source install/setup.bash
colcon build --packages-select invictasim
```

## Run

From workspace root:

```sh
source install/setup.bash
ros2 launch invictasim invictasim.launch.py
```

Or run the node directly:

```sh
source install/setup.bash
ros2 run invictasim invictasim
```

## Keyboard Mode

Set in [config/invictasim/global.yaml](../../config/invictasim/global.yaml):

```yaml
invictasim:
  input_adapter: "keyboard"
```

Controls and UI:

- `W`: increase throttle
- `S`: brake/reverse throttle
- `A`: steer left
- `D`: steer right
- `Q` or window close: exit keyboard input window
- SDL window shows labeled throttle and steering bars plus a focus hint

Keyboard implementation:

- [src/io/input/keyboard.cpp](src/io/input/keyboard.cpp)

## ROS Input Mode

Set in [config/invictasim/global.yaml](../../config/invictasim/global.yaml):

```yaml
invictasim:
  input_adapter: "ros"
```

Expected input topic:

- `/control/command` (`custom_interfaces/msg/ControlCommand`)

ROS input implementation:

- [src/io/input/ros.cpp](src/io/input/ros.cpp)

## Tests

Run tests for this package:

```sh
source install/setup.bash
colcon test --packages-select invictasim
colcon test-result --all --verbose
```