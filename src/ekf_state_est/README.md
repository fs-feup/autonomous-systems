# EKF State Estimation Package

The State Estimation module is responsible for the processing of the perception module and localisation sensors' data and its transformation into useful information. In this case, the data is combined to generate the best estimates possible on the pose and speed of the vehicle and the location of the cones that delimit the track.

## Package Info

- **Node name:** ekf_state_est
- **Package name:** ekf_state_est

### Launch files

- **eufssim-slam.launch.py** - EUFS SIM simulator adapter and SLAM with EKF
- **pacsim-slam.launch.py** - PacSim simulator adapter and SLAM with EKF
- **normal-slam.launch.py** - in vehicle SLAM with EKF

## Design

Below, some diagrams are presented that can illustrate the structure and behaviour of the program.

### Behaviour

The control flow below illustrates briefly the flow of execution and information through the node, you can also checkout [documentation pdf](../../docs/assets/Loc_map/EKFSlam1.0.pdf) to get a better understanding.

![Control Flow Diagram](../../docs/diagrams/state_estimation/control-flow.drawio.svg)

The sequence diagrams below illustrate in greater detail the interactions between the components of the system in a normal use case.

![Sequence Diagram](../../docs/diagrams/state_estimation/sequence.drawio.svg)

1. Information from localisation sensors is received in the adapter (the adapter is a layer for adaptation of the different environments the node runs in).
2. LMNode (the localisation and mapping node) processes the information and requests execution of the prediction step to the Extended Kalman Filter cass (illustrated below).
    1. The Motion Model is used to utilize the Motion data to predict the change in state.
    2. The Motion Model is used to reaturn the G matrix (maps the transformation from the motion to the state)
    3. Get the noise matrix, required for the calculation of the predicted covariance matrix P.
    4. Compute predicted X and P.
3. After the step, the pose and map are updated and published.
4. When cone coordinates are published by perception, the LMNode receives them directly, executing the perception_callback.
5. The node requests execution of the correction step to the EKF (more detail below). For every landmark (cone):
    1. Discovery is executed, checking if the observation corresponds to a new cone/landmark or one that already exists in the state vector (X)
    2. H (matrix that maps the transformation from the state to the observations), Q (2x2 matrix that encodes the perception's noise) are requested to the Observation Model (class that encodes how perception data is interpreted).
    3. the kalman gain (K) is calculated (smart weight that controls in what extent observations contribute to the update of the state).
    4. the observation model is executed to obtain the z_hat, required for the calculation of the state.
    5. Compute X and P.
6. The EKF updates the estimates and the node publishes them.

### Structure

The node is composed by multiple classes. The diagram below illustrates roughly how they sit in the code structure. The main two are the LMNode, which is the class of the node, and the Extended Kalman Filter, which corresponds to the code of the implementation of the EKF SLAM.

![Class Diagram](../../docs/diagrams/state_estimation/class.drawio.svg)

Take a look at the [Doxygen generated documentation](https://fs-feup.github.io/autonomous-systems/dir_388d5df3221aab46ce4275c3697a683f.html).

## Full Documentation

More precise documentation can be found [here](https://github.com/fs-feup/documentation/blob/main/AS/FSFEUP_02_State_Estimation/main.pdf).

## Main External Libraries

- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [ROS2](https://docs.ros.org/en/foxy/index.html)
- [Gtest](http://google.github.io/googletest/)