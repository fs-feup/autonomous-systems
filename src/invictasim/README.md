# Invictasim Package

## Package Information

### Description

This package contains the invictasim simulator built by Formula Student FEUP for the autonomous system. 

### Features

- **Vehicle Model:** Bicycle kinematic and dynamic model with configurable tire, aerodynamic, and drivetrain parameters
- **Real-time Simulation:** 1000 Hz simulation loop with real-time synchronization
- **ROS 2 Integration:** Publishes simulation state and accepts control inputs via ROS topics

### Launch Configurations

- [invictasim.launch.py](launch/invictasim.launch.py): Launches the invictasim simulator node with configuration from workspace config files.

### Configuration

All configuration files are located in `/home/ws/config/invictasim/`:

**Main Simulator Configuration:**
- [config.yaml](/home/ws/config/invictasim/config.yaml) - Simulator parameters:
  - Vehicle model selection
  - Simulation timestep (default: 0.001s = 1000 Hz)

**Global Configuration (Workspace):**
- [/home/ws/config/global/global_config.yaml](/home/ws/config/global/global_config.yaml) - System-wide parameters:
  - Discipline (autocross, skidpad, acceleration)
  - Track name and vehicle frame ID
  - Simulation speed ratio (1.0 = real-time)

## How to Run

### Install Dependencies

```sh
./dependencies_install.sh
```

### Compiling

```sh
colcon build --packages-up-to invictasim
```

### Testing

```sh
colcon test --packages-select invictasim
```

To check test results:
```sh
colcon test-result --all --verbose
```

### Running

Use a launch file:

```sh
source ./install/setup.bash # If in a new terminal
ros2 launch invictasim invictasim.launch.py
```

or run directly:

```sh
source ./install/setup.bash # If in a new terminal
ros2 run invictasim invictasim
```