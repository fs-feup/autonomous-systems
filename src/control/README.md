# Control Module

The Control node uses Planning (pathpoints with expected velocity values) and State Estimation (current vehicle pose) data to effectively calculate longitudinal (torque) and lateral values (steering angle) using PID and Pure Pursuit controllers, respectively. 

## Dependencies

* custom_interfaces
* pacsim
* fs_msgs
* eufs_msgs

## Compiling

From src folder:
```sh
colcon build --packages-up-to control
```

## Testing

From src folder:
```sh
colcon test --packages-select control
```

## Running
Run with ros2 launch. You may select custom parameters for testing such as simulated state estimation or the mocker node. Check the [launch file](./launch/control.launch.py) for more details. Example:
```bash
ros2 launch control control.launch.py 'adapter:=pacsim' 'use_simulated_se:=true' 'mocker_node:=true'
```
Using simulated state estimation allows you to independently test control without using the state estimation module, by using information directly from the simulator. The mocker node allows for the same thing but for planning, by using a hand-picked pathpoint array. The lookahead_gain is for tuning purposes.

## Design
The following class diagram illustrates the structure of the Control package:

![Class Diagram](../../docs/assets/Control/controlClassDiagram.drawio.png)
We have the following classes:    
* Control: The node itself and the main class that initializes the other classes and runs the control callback.
* PSolver: The class that receives Planning information and calculates the reference for the controllers.
* PID: The class that implements the PID controller, for Longitudinal Control.
* PP: The class that implements the Pure Pursuit controller, for Lateral Control.
* Adapter: Provides an interface to correctly subscribe/publish, as well as parse and normalize values as needed by the different simulators we use.

The actions taken when the node gets spinning are the following:
![Sequence Diagram](../../docs/assets/Control/ControlSequenceDiagram.drawio.png)