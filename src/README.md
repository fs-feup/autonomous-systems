# Src Folder

This directory contains the code for the main AS Pipeline. 

- **control** -> longitudinal/lateral control
- **planning** -> path planning
- **ekf_state_est** -> localisation and mapping
- **perception** -> perception
- **inspection** -> static inspection package
- **mocker_node** -> package for a node that is a path planning mock
- **evaluator** -> package for the node that calculates metrics of the system
- **common_lib** -> common library for the AS pipeline
- **pacsim_keys** -> package for controlling the PacSim vehicle with the keyboard
- **power_log** -> package for logging the power consumption of the computer
- **launcher** -> package for launching the AS pipeline, with multiple launch configurations
- **dashboard** -> package for the dashboard for plotting the evaluation metrics
- **cloud_storage** -> package for storing the evaluation metrics in the cloud

All code for the main pipeline is developed in ROS2 packages and is meant to be developed inside the teams' dev container environment.
