# Evaluator Package

## Package Information

### Description

This package contains the evaluator node, used to evaluate the AS main pipeline, with aid of simulation environments and other data.

### Folder Structure

- [adapter](evaluator/adapter.py): Adapter base class to change ROS2 interfaces according to the simulator or environment
- [pacsim_adapter](evaluator/pacsim_adapter.py): Adapter for PacSim simulator
- [eufs_adapter](evaluator/eufs_adapter.py): Adapter for EUFS simulator
- [fsds_adapter](evaluator/fsds_adapter.py): Adapter for FSDS simulator
- [vehicle_adapter](evaluator/vehicle_adapter.py): Adapter for the vehicle
- [formats](evaluator/formats.py): Functions for formatting messages into a common format
- [metrics](evaluator/metrics.py): Functions for metrics calculation
- [adapter_maps](evaluator/adapter_maps.py): Maps for adapter configurations
- [evaluator](evaluator/evaluator.py): Evaluator node class


### Launch Configurations

- [evaluator-eufs.launch.py]: evaluator with adapters for EUFS SIM
- [evaluator-pacsim.launch.py]: evaluator with adapters for PacSim
- [evaluator-fsds.launch.py]: evaluator with adapters for FSDS

## How to Run

### Install Dependencies

```sh
./denpendencies_install.sh
```

### Compiling

```sh
colcon build --packages-up-to evaluator
```

### Testing

```sh
colcon test --packages-select evaluator # use event-handler=console_direct+ for imediate output
```

To check test results:
```sh
colcon test-result --all --verbose
```

### Running

Use a launch file:

```sh
source ./install/setup.bash # If in a new terminal
ros2 launch evaluator evaluator-eufs.launch.py
```

or run directly:

```sh
source ./install/setup.bash # If in a new terminal
ros2 run evaluator evaluator
```

## Design

The structure of the node is quite simple and depicted by the following class diagram:

![Class Diagram](../../docs/diagrams/sim-inf/evaluator-class-diagram.drawio.svg)

The way the system works can be explained with the aid of the following sequence diagram:

![Sequence Diagram](../../docs/diagrams/sim-inf/evaluator-sequence-diagram.drawio.svg)

The Evaluator node class only creates Subscribers for the topics intrinsic to the system. The subscriptions and subsequent linkage to callbacks is all made in the adapters, as they depend of the simulator topics. The callbacks in the adapters simply organize and format the inputs to a standard format, so that metrics can be calculated and published by the evaluator node and its functions. 

This is true for all but the control node, which for simplicity reasons sends all the information necessary for evaluation in a separate message, as it does not use simulation data to evaluate itself.

The concept of the complete evaluation system can be seen in the following diagram:

![System Diagram](../../docs/diagrams/sim-inf/evaluation-system.drawio.svg)

### Results

The metrics will be published in ```/evaluator/*``` topics.