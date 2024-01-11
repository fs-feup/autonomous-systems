# Technologies

This document compiles the versions of all tools and technologies used in this project.

## Tools

- [Ubuntu 22.04](https://releases.ubuntu.com/focal/)
- [ROS2 Humble desktop](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Cpplint 1.6.1](https://github.com/cpplint/cpplint) - C++ only
- [Cppcheck 1.90](https://cppcheck.sourceforge.io/) - C++ only
- [Clangformat 10.0.0-4ubuntu1](https://clang.llvm.org/docs/ClangFormat.html) - C++ only
- [Doxygen 1.9.6](https://doxygen.nl/download.html)
- [Ruff 0.0.254](https://beta.ruff.rs/docs/configuration/#using-pyprojecttoml), [git](https://github.com/charliermarsh/ruff) - Python only

## Libraries

### In use

- [rclpy](https://docs.ros2.org/foxy/api/rclpy/index.html)
- [rclcpp](https://docs.ros2.org/latest/api/rclcpp/)
- [gtest](https://google.github.io/googletest/)
- [unittest](https://docs.python.org/3/library/unittest.html)
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
Note: these all come with ROS2 or are already in the project

- [libcgal](https://www.cgal.org/)
- [libgsl](https://www.gnu.org/software/gsl/)

To get an insight on the all the libraries to be installed, check the [dependencies installation script](../src/dependencies_install.sh).

### RCL - ROS Client Library

The RCL stands for ROS Client Library and is the main interaction form with the ROS2 environment. RCL has two different implementations for each language supported by ROS: python (rclpy) and c++ (rclcpp). 
The RCL interface is implemented in more than those languages. However, the rest of the libraries are community maintained. 
Essentially this library allows for programming of custom nodes and communication mechanisms between them. 

#### Important links:

- [Documentation page](https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Client-Libraries.html)
- [rclcpp repo](https://github.com/ros2/rclcpp)
- [rclpy repo](https://github.com/ros2/rclpy)
- [QoS explanation](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html)
- [QoS class documentation](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1QoS.html#ad7e932d8e2f636c80eff674546ec3963)
- [Simple Publisher Subscriber Tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)

### Eigen

[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) is a c++ library for efficient linear algebra operations. It is used for eficient matrix calculations, for example, in the Kalman Filter.

### Google Test

Google test is a c++ unit test framework. Here is a [Tutorial](https://www.youtube.com/watch?v=JJqRlSTQlh4&t=1453s).

### Libcgal and libgsl

GNU libraries for computational geometry and numerical operations used for Planning.

## Languages

- [python](https://www.python.org/)
- [c++](https://cplusplus.com/)

Installation guides for all this software can be found in their respective [tutorials](./tutorials/).