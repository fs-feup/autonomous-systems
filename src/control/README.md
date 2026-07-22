# Control Module

## Package Information

### Description

The Control node uses Planning (pathpoints with expected velocity values) and State Estimation (current vehicle pose and velocity) data to calculate longitudinal (throttle) and lateral (steering angle) controls. 


### Folder Structure

- [adapter](./include/adapter/): Adapters to change ros2 interfaces according to simulator or environment
- [config](./include/config/): Parameters struct definition, which will be read from the [config](../../config/control/) folder
- [controller](./include/controller/): algorithms that calculate both lateral and longitudinal controls (full control solvers)
- [lateral_controller](./include/lateral_controller/): algorithms that calculate lateral control only (steering angle)
- [longitudinal_controller](./include/longitudinal_controller/): algorithms that calculate longitudinal control only (throttle)
- [ros_node](./include/ros_node/): ROS2 Node class
- [utils](./include/utils/): helper functions used in the other folders


### Important Dependencies

## How to Run

### Install Dependencies

Both in /autonomous_systems and /src/control.

```sh
  ./dependencies_install.sh
```

### Compiling

```sh
colcon build --packages-up-to control
```

### Testing

```sh
colcon test --packages-select control # use event-handler=console_direct+ for imediate output
```

To check test results:
```sh
colcon test-result --all --verbose
```
### Running

Run with ros2 launch. Example:

```sh
source ./install/setup.bash # If in a new terminal
ros2 launch control control.launch.py
```

To configure the node's inputs and environment, use the [global config file](../../config/global/global_config.yaml). Using simulated state estimation (use_simulated_se) allows you to independently test control without using the pose estimate from the state estimation module, by using information directly from the simulator. The mocker node allows for the same thing but for planning (use_simulated_planning), by using a hand-picked pathpoint array. To do the same thing for velocities use use_simulated_velocities. To run the node in different environments, use the adapter parameter.

To tune the node's parameters, use the [control node's config file](../../config/control/) for the environment you're using.

## Design
The following class diagram illustrates the structure of the Control package:

![Class Diagram](../../docs/assets/Control/control_class_diagram.drawio.png)

We have the following classes:    
* ControlNode: The node abstract class itself, from which adapters inherit. This is the main class that initializes the other classes and interacts with ROS2.
* Adapters: Provide an interface to correctly subscribe/publish, as well as parse and normalize values as needed by the different simulators we use and for the actual vehicle.
* Controller: The generic controller, from which other full controllers must inherit.
* DecoupledController: The generic decoupled controller, i.e. the lateral and longitudinal controllers work independently of each other.
* LateralController: The generic lateral controller, from which other lateral controllers must inherit.
* LongitudinalController: The generic longitudinal controller, from which other longitudinal controllers must inherit.
* PID: The class that implements the PID controller, for Longitudinal Control.
* PurePursuit: The class that implements the Pure Pursuit controller, for Lateral Control.
