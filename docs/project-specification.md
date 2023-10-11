# Project Specifications

This project aims to develop an Aritificial Inteligence system for a computer to be installed in a car, with the objective of making said vehicle autonomous in the driving tasks pruposed in multiple Formula Student competitions.

## Technologies

The main technologies used are:
- ROS2 (Robot Operating System 2)
- C++ and Python

Other technologies used in this project:
- Cppcheck
- Doxygen
- CMake
- Google Tests
- ...

Refer to [technologies file](./technologies.md) for more.

## Architecture

The AI computer is divided into 4 main components:
- Perception - responsible for converting the visual sensor data into detections of landmarks (cones) in the track
- Localization and Mapping - responsible for generating a map of the track of path planning to use and estimate its position in the map
- Path Planning - plan a path for the vehicle to follow
- Control - traduce the designed path into controls to the vehicle

The following components diagram illustrates the structure of the system.

![Components Diagram](./assets/architecure.drawio.svg)

The activity diagram / flow chart below illustrates the behaviour of the system and the flow of information through it.

![Activity Diagram](./assets/system-flow-chart.drawio.svg)