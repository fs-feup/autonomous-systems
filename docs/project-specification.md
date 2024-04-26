# Project Specifications - OLD

This project aims to develop an Artificial Inteligence system for a computer to be installed in a car, with the objective of making said vehicle autonomous in the driving tasks pruposed in multiple Formula Student competitions.

## Technologies

### Tools

- [Ubuntu 22.04](https://releases.ubuntu.com/focal/)
- [ROS2 Humble desktop](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Clangformat 10.0.0-4ubuntu1](https://clang.llvm.org/docs/ClangFormat.html) - C++ only

### Libraries

#### In use

- [rclpy](https://docs.ros2.org/foxy/api/rclpy/index.html)
- [rclcpp](https://docs.ros2.org/latest/api/rclcpp/)
- [gtest](https://google.github.io/googletest/)
- [unittest](https://docs.python.org/3/library/unittest.html)
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) - [Documentation](https://eigen.tuxfamily.org/dox/group__TutorialSTL.html)
- [PCL](https://pointclouds.org)

- [libcgal](https://www.cgal.org/)
- [libgsl](https://www.gnu.org/software/gsl/)

To get an insight on the all the libraries to be installed, check the [dependencies installation script](../dependencies_install.sh).

#### RCL - ROS Client Library

The RCL stands for ROS Client Library and is the main interaction form with the ROS2 environment. RCL has two different implementations for each language supported by ROS: python (rclpy) and c++ (rclcpp). 
The RCL interface is implemented in more than those languages. However, the rest of the libraries are community maintained. 
Essentially this library allows for programming of custom nodes and communication mechanisms between them. 

Important links:

- [Documentation page](https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Client-Libraries.html)
- [rclcpp repo](https://github.com/ros2/rclcpp)
- [rclpy repo](https://github.com/ros2/rclpy)
- [QoS explanation](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html)
- [QoS class documentation](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1QoS.html#ad7e932d8e2f636c80eff674546ec3963)
- [Simple Publisher Subscriber Tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)

#### Eigen

[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) is a c++ library for efficient linear algebra operations. It is used for eficient matrix calculations, for example, in the Kalman Filter.

#### Google Test

Google test is a c++ unit test framework. Here is a [Tutorial](https://www.youtube.com/watch?v=JJqRlSTQlh4&t=1453s).

#### Libcgal and libgsl

GNU libraries for computational geometry and numerical operations used for Planning.

### Languages

- [python](https://www.python.org/)
- [c++](https://cplusplus.com/)

Installation guides for all this software can be found in their respective [tutorials](./tutorials/).

## Architecture

The AI computer is divided into 4 main components:
- Perception - responsible for converting the visual sensor data into detections of landmarks (cones) in the track
- EKF SLAM - responsible for generating a map of the track of path planning to use and estimate its position in the map
- KISS ICP - LiDAR odometry 
- Planning - plan a path for the vehicle to follow
- Longitudinal Control - traduce the designed path into throttle controls to the vehicle
- Lateral Control - traduce the designed path into steering controls

The following components diagram illustrates the structure of the system.

![Components Diagram](./assets/architecture.svg)

The activity diagram / flow chart below illustrates the behaviour of the system and the flow of information through it.

![Activity Diagram](./assets/behaviour.svg)