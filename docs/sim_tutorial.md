# ROS2 and EUFS SIM

This tutorial is meant to guide on the setup and installation of the [EUFS Simulator](https://gitlab.com/eufs/eufs_sim), as well as give insight on [ROS2](https://docs.ros.org/en/foxy/index.html)’s most important functionalities and the way ROS2 is integrated into the simulator.

# Table of Contents

# Links

- [Youtube ROS2 tutorials](https://www.youtube.com/watch?v=uYW8UJZTuAg&list=PLRE44FoOoKf7NzWwxt3W2taZ7BiWyfhCp&index=6)
- [EUFS SIM repo](https://gitlab.com/eufs/eufs_sim)
- [ROS2 galactic documentation](https://docs.ros.org/en/galactic/index.html)
- [SSH to Virtual Machine](https://averagelinuxuser.com/ssh-into-virtualbox/)

# Keywords

- ROS - Robot Operating System
- ROS2 - New version of ROS that will be used
- ROS and ROS2 are also used to mention the system behind both that is common to one another
- CLI - Command Line Interface
- SIM - Simulator

# EUFS SIM - Installation and Setup

The [official GitLab repository](https://gitlab.com/eufs/eufs_sim) already has a fairly complete guide, having only some few details unmentioned or out of date.

## Prerequesites

- [Ubuntu 20.04 (LTS)](https://releases.ubuntu.com/focal/)
- [ROS2 Galactic Desktop](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
- [Colcon](https://colcon.readthedocs.io/en/released/user/installation.html)
- Rosdep

## Ubuntu Setup

If you do not own a machine that runs on Ubuntu 20.04, the best option is to set up a virtual machine. To do this, you can use [VirtualBox](virtualbox.org).

**Steps:**

- Download Ubuntu 20.04 image ([link](https://releases.ubuntu.com/focal/ubuntu-20.04.5-desktop-amd64.iso))
- Install VirtualBox - [Ubuntu](https://www.virtualbox.org/wiki/Linux_Downloads), [Windows/Mac](https://www.virtualbox.org/wiki/Downloads)
- Setup Ubuntu Virtual Machine - [youtube tutorial](https://www.youtube.com/watch?v=IOwlnpWPuj0)

[https://www.youtube.com/watch?v=IOwlnpWPuj0](https://www.youtube.com/watch?v=IOwlnpWPuj0)

## ROS2 Setup

**ROS (Robot Operating System)** is a tool that allows engineers to easily program robots, providing a system to integrate programs in the robot as well as very handy tools that would otherwise required implementation. **ROS2** is a new version of ROS with a few improvements, but the same core idea. You can check the difference between the two version [here](https://roboticsbackend.com/ros1-vs-ros2-practical-overview/). 

Both ROS and ROS2 are organized into **distributions** (releases). These distributions mostly vary in the packages available for usage and their implementations. The distribution that FS FEUP will use is the same used in the EUFS SIM project, which is **ROS2-Galactic**. This distribution is in EOL (End-Of-Life), meaning it will not receive any further updates. In any case, this is the system to use for now. 

To download and setup this distribution, you can mostly follow the setup in their [documentation website](http://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html). This guide in a simplified manner:

### Set LOCALE

Make sure you have a locale which supports `UTF-8`. If you are in a minimal environment (such as a docker container), the locale may be something minimal like `POSIX`. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### Setup Sources

You will need to add the ROS 2 apt repository to your system.

First ensure that the [Ubuntu Universe repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Now add the ROS 2 GPG key with apt.

```bash
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add repository to your sources list.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS2 packages

Update your apt repository caches after setting up the repositories.

```bash
sudo apt update
sudo apt upgrade
```

**Desktop Install:** ROS, RViz, demos, tutorials.

```bash
sudo apt install ros-galactic-desktop
```

### Environment Setup

Run every time you open your shell

```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/galactic/setup.bash
```

Or edit your shell rc file

```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
```

### Some examples

In one terminal, source the setup file and then run a C++ `talker`:

```bash
source /opt/ros/galactic/setup.bash
ros2 run demo_nodes_cpp talker
```

In another terminal source the setup file and then run a Python `listener`:

```bash
source /opt/ros/galactic/setup.bash
ros2 run demo_nodes_py listener
```

You should see the `talker`
 saying that it’s `Publishing`
 messages and the `listener`
 saying `I heard`
 those messages. This verifies both the C++ and Python APIs are working properly. Hooray!

## Colcon setup

[Colcon](https://colcon.readthedocs.io/en/released/) is a **command line tool** with the goal of improving building, testing and the usage of multi-package software applications in general. Essentially, it is a great **build tool** to help manage ROS2 packages.

### Setup

```bash
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install python3-colcon-common-extensions
```

Install pip3 if you haven’t

```bash
sudo apt install python3-pip
```

```bash
pip3 install colcon-common-extensions -U
```

## EUFS SIM Repository Setup

To use the simulator from **EUFS,** you need to clone two repositories into the same base folder. To do this, run these commands:

```bash
cd ~/Documents
mkdir eufs_sim
cd eufs_sim
curl "https://gitlab.com/eufs/eufs_msgs/-/archive/master/eufs_msgs-master.zip" --output "eufs_msgs-master.zip" && unzip eufs_msgs-master.zip
rm eufs_msgs-master.zip
curl "https://gitlab.com/eufs/eufs_sim/-/archive/master/eufs_sim-master.zip" --output "eufs_sim-master.zip" && unzip eufs_sim-master.zip
rm eufs_sim-master.zip
```

Set the path of this directory as the EUFS_MASTER environment variable.

```bash
# /path/to/the/directory would be ~/Documents/eufs_sim in the case above
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
echo 'export EUFS_MASTER=/path/to/the/directory' >> ~/.bashrc
source ~/.bashrc
```

## Rosdep Setup and Dependency Installation

**Rosdep** is a command line tool that helps install ROS projects’ dependencies. 

To install and setup rosdep:

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update --include-eol-distros # without sudo
```

To install project dependencies:

```bash
rosdep install --from-paths $EUFS_MASTER --ignore-src -r -y
```

## Compilation and running

To compile EUFS SIM project:

```bash
cd [workspace]
colcon build
```

To run the project:

```bash
cd [workspace]
source install/setup.bash
ros2 launch eufs_launcher eufs_launcher.launch.py
```

# ROS2 basics

To understand how the simulator works, we must first understand what ROS2 is and how to use it. 

## ROS2 System Overview

### What ROS offers

- an environment where your programs can communicate with each other
- handy robotics tools/algorithms
- other robotics related material online

### ROS Environment

The environment ROS programs run on works similarly to a distributed computers system, where the computers are, in this case, programs/processes. The system is then composed by the **nodes** (the ROS programs/scripts you are running) which have three distinct forms to communicate:

- **Topics -** the nodes communicate in a [Publish/Subscribe](https://www.enjoyalgorithms.com/blog/publisher-subscriber-pattern) paradigm, where nodes can publish events (data) to topics and subscribe topics so that they receive events when they are published
- **Services -** the nodes communicate via a [Client/Server](https://en.wikipedia.org/wiki/Client%E2%80%93server_model) paradigm, where client nodes request data to server nodes in a blocking synchronous fashion (the client awaits the server’s response)
- **Actions -** the nodes communicate via a [Client/Server](https://en.wikipedia.org/wiki/Client%E2%80%93server_model) paradigm, where client nodes request data to server nodes in a non-blocking asynchronous fashion (the client does not wait for the response). Actions are built on top of both topics and services.

![ROS2 Environment Schema](https://user-images.githubusercontent.com/123366776/216478136-cf1e8144-f48b-415e-b618-d2f514480d3c.png)

### ROS2 Programming

ROS2 projects are organized into packages. Packages are composed by software for one or more nodes. Inside a package, the tendency is to use more or less the same technologies. Multi-package ROS2 projects can be composed of code in multiple different languages and using multiple different build systems. Essentially, a package defines norms for the code written to be traduced into ROS components (TODO: see if this naming is appropriate). 

ROS2 mainly utilizes 2 build systems:

- Python - originates project in Python
- CMake - originates project in C++

There are more languages to which ROS and ROS2 are available. However, these languages’ implementation of ROS is community made and maintained.

ROS2 offers a multitude of tools in the form of code libraries for these programming languages.

### ROS2 CLI (Command Line Interface)

ROS2 also offers a CLI that provides the user with the option to interact with the ROS2 system outside a program, using only a Shell. For instance, you can publish data into a topic through the command line, like a ROS node would, thus interacting with the system. This tool will come in handy on the time of testing and debugging systems, as well as punctual interactions.

## Topics, Services and Actions

In a ROS2 system, nodes can interact through 3 different types of interfaces: ************Actions, Services************ and ************Topics.************ Each of these interfaces have strengths and weaknesses, meaning there is a correct situation to use each of them. According to [documentation](https://docs.ros.org/en/foxy/How-To-Guides/Topics-Services-Actions.html):

### Topics

- System works in a **Publish/Subscribe** fashion
- The publisher decides when data is sent
- The subscriber receives the data when it is available
- **Many-to-Many relation**
- **USAGE:** Good for continuous data streams (sensor data, robot state, …) and real time communication; when there is no needed feedback

### Services

- System works in a **Client/Server (Request/Response)** fashion
- Simple blocking call, client blocks waiting for response
- **USAGE:** Should be used sparingly, good for remote procedure calls that terminate quickly

### Actions

- Built on top of Topics and Services
- Composed by **Goal**, **Feedback** and **Result.** Client node sends a goal to server node, who acknowledges the goal and returns a stream of feedback and the result (if returnable)
- System works in a **Client/Server** fashion but can keep long-lived connections and connections are preemptable (can be canceled)
- More complex implementation
- Can keep state of a goal
- More complex non-blocking background processing; used for longer tasks like execution of robot actions
- **USAGE:** Should be used for discrete behavior that runs for a longer time and that require feedback during execution;

## Nodes, Packages and Executables

**Nodes** are the building block of a ROS system. Nodes can be seen as an entity of the system. **Packages** are more like projects. A Package can define different elements of a ROS system, such as Topics, Nodes, etc. A Package is essentially a bundle of ROS2 resources with defined technologies. An **Executable** is not the same as a Node. In ROS2, an executable is a compiled program that can launch one (typically) or more nodes into the system.

## ROS2 CLI (Command Line Interface)

### Links

- [ROS2 CLI Cheat Sheet](https://github.com/ubuntu-robotics/ros2_cheats_sheet/blob/master/cli/cli_cheats_sheet.pdf)

ROS2 comes with a powerful CLI that enables the user to perform multiple actions. To get information on all commands, we can use ‘ros2’ with the help flag:

```bash
ros2 -h
```

**OUTPUT:**

- action     Various action related sub-commands
- bag        Various rosbag related sub-commands
- component  Various component related sub-commands
- daemon     Various daemon related sub-commands
- doctor     Check ROS setup and other potential issues
- interface  Show information about ROS interfaces
- launch     Run a launch file
- lifecycle  Various lifecycle related sub-commands
- multicast  Various multicast related sub-commands
- node       Various node related sub-commands
- param      Various param related sub-commands
- pkg        Various package related sub-commands
- run        Run a package specific executable
- security   Various security related sub-commands
- service    Various service related sub-commands
- topic      Various topic related sub-commands
- wtf        Use `wtf` as alias to `doctor`

### Node

Node command gives information on, you guessed it, nodes:

```bash
ros2 node -h
# usage: ros2 node [-h] Call `ros2 node <command> -h` for more detailed usage. ...
#
# Various node related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#
# Commands:
#   info  Output information about a node
#   list  Output a list of available nodes
#
#   Call `ros2 node <command> -h` for more detailed usage.
```

Node info command compacts a lot of information related to the topics the node interacts with and services and actions. **Example:**

```bash
ros2 node /turtlesim info
# /turtlesim
#   Subscribers:
#     /parameter_events: rcl_interfaces/msg/ParameterEvent
#     /turtle1/cmd_vel: geometry_msgs/msg/Twist
#   Publishers:
#     /parameter_events: rcl_interfaces/msg/ParameterEvent
#     /rosout: rcl_interfaces/msg/Log
#     /turtle1/color_sensor: turtlesim/msg/Color
#     /turtle1/pose: turtlesim/msg/Pose
#   Service Servers:
#     /clear: std_srvs/srv/Empty
#     /kill: turtlesim/srv/Kill
#     /reset: std_srvs/srv/Empty
#     /spawn: turtlesim/srv/Spawn
#     /turtle1/set_pen: turtlesim/srv/SetPen
#     /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
#     /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
#     /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
#     /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
#     /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
#     /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
#     /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
#     /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
#   Service Clients:
#
#   Action Servers:
#     /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
#   Action Clients:
```

### Interface

Interface command allows the user to get information on the format of messages for the 3 different types of interfaces, as well as the interfaces defined in a packages, etc.

```bash
ros2 interface -h
# usage: ros2 interface [-h]
#                       Call `ros2 interface <command> -h` for more detailed
#                       usage. ...
# 
# Show information about ROS interfaces
# 
# optional arguments:
#   -h, --help            show this help message and exit
# 
# Commands:
#   list      List all interface types available
#   package   Output a list of available interface types within one package
#   packages  Output a list of packages that provide interfaces
#   proto     Output an interface prototype
#   show      Output the interface definition
# 
#   Call `ros2 interface <command> -h` for more detailed usage.
```

To get information, for instance, on the format of the requests of a certain service:

```bash
ros2 interface show <service_name>
# or
ros2 interface proto <service_name>
```

‘interface show’ shows the type definitions of the parameters while proto shows a string format of the message. **Example:**

```bash
# Check the messages for /cmd topic 
ros2 topic info /cmd
# OUTPUT
# Type: ackermann_msgs/msg/AckermannDriveStamped
# Publisher count: 1
# Subscription count: 1
# Check the message format
ros2 interface proto ackermann_msgs/msg/AckermannDriveStamped
# OUTPUT
# "header:
#   stamp:
#     sec: 0
#     nanosec: 0
#   frame_id: ''
# drive:
#   steering_angle: 0.0
#   steering_angle_velocity: 0.0
#   speed: 0.0
#   acceleration: 0.0
#   jerk: 0.0
# "
```

### Pkg

Pkg command is useful to find out what packages you have installed, as well as information on specific packages.

```bash
ros2 pkg -h
# usage: ros2 pkg [-h] Call `ros2 pkg <command> -h` for more detailed usage. ...
#
# Various package related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#
# Commands:
#   create       Create a new ROS 2 package
#   executables  Output a list of package specific executables
#   list         Output a list of available packages
#   prefix       Output the prefix path of a package
#   xml          Output the XML of the package manifest or a specific tag
#
#   Call `ros2 pkg <command> -h` for more detailed usage.
```

If you are wondering what you might be able to run in a package, you can use **executables** command. **Example:**

```bash
ros2 pkg executables turtlesim
# turtlesim draw_square
# turtlesim mimic
# turtlesim turtle_teleop_key
# turtlesim turtlesim_node
```

### Topic

Topic command allows the user to attain information on topics, as expected.

```bash
ros2 topic -h
# usage: ros2 topic [-h] [--include-hidden-topics]
#                   Call `ros2 topic <command> -h` for more detailed usage. ...
# 
# Various topic related sub-commands
#
# optional arguments:
#   -h, --help            show this help message and exit
#   --include-hidden-topics
#                         Consider hidden topics as well

# Commands:
#   bw     Display bandwidth used by topic
#   delay  Display delay of topic from timestamp in header
#   echo   Output messages from a topic
#   find   Output a list of available topics of a given type
#   hz     Print the average publishing rate to screen
#   info   Print information about a topic
#   list   Output a list of available topics
#   pub    Publish a message to a topic
#   type   Print a topic's type
#
#   Call `ros2 topic <command> -h` for more detailed usage. 
```

topic info command shows the publishers and subscribers (nodes) of the topic, as well as the type (message format name). **Example:**

```bash
ros2 topic list
# /parameter_events
# /rosout
ros2 topic info /parameter_events
# Type: rcl_interfaces/msg/ParameterEvent
# Publisher count: 1
# Subscription count: 0
```

topic pub command is useful to interact with the ROS environment without the need of a program. **Example:**

```bash
# node /turtlesim is running
ros2 node /turtlesim info
# /turtlesim
#   Subscribers:
#     /parameter_events: rcl_interfaces/msg/ParameterEvent
#     /turtle1/cmd_vel: geometry_msgs/msg/Twist
#   Publishers:
#     /parameter_events: rcl_interfaces/msg/ParameterEvent
#     /rosout: rcl_interfaces/msg/Log
#     /turtle1/color_sensor: turtlesim/msg/Color
#     /turtle1/pose: turtlesim/msg/Pose
#   Service Servers:
#     /clear: std_srvs/srv/Empty
#     /kill: turtlesim/srv/Kill
#     /reset: std_srvs/srv/Empty
#     /spawn: turtlesim/srv/Spawn
#     /turtle1/set_pen: turtlesim/srv/SetPen
#     /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
#     /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
#     /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
#     /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
#     /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
#     /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
#     /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
#     /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
#   Service Clients:
#
#   Action Servers:
#     /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
#   Action Clients:
ros2 interface proto geometry_msgs/msg/Twist
# "linear:
#   x: 0.0
#   y: 0.0
#   z: 0.0
# angular:
#   x: 0.0
#   y: 0.0
#   z: 0.0
# "
ros2 topic pub -h
# usage: ros2 topic pub [-h] [-r N] [-p N] [-1 | -t TIMES] [--keep-alive N] [-n NODE_NAME]
#                       [--qos-profile {unknown,system_default,sensor_data,services_default,parameters,parameter_events,action_status_default}]
#                       [--qos-depth N] [--qos-history {system_default,keep_last,keep_all,unknown}]
#                       [--qos-reliability {system_default,reliable,best_effort,unknown}]
#                       [--qos-durability {system_default,transient_local,volatile,unknown}]
#                       topic_name message_type [values]
#
# Publish a message to a topic
#
# positional arguments:
#   topic_name            Name of the ROS topic to publish to (e.g. '/chatter')
#   message_type          Type of the ROS message (e.g. 'std_msgs/String')
#   values                Values to fill the message with in YAML format (e.g. 'data: Hello World'),
#                         otherwise the message will be published with default values
#
# optional arguments:
#   -h, --help            show this help message and exit
#   -r N, --rate N        Publishing rate in Hz (default: 1)
#   -p N, --print N       Only print every N-th published message (default: 1)
#   -1, --once            Publish one message and exit
#   -t TIMES, --times TIMES
#                         Publish this number of times and then exit
#   --keep-alive N        Keep publishing node alive for N seconds after the last msg (default: 0.1)
#   -n NODE_NAME, --node-name NODE_NAME
#                         Name of the created publishing node
#   --qos-profile {unknown,system_default,sensor_data,services_default,parameters,parameter_events,action_status_default}
#                         Quality of service preset profile to publish with (default:
#                         system_default)
#   --qos-depth N         Queue size setting to publish with (overrides depth value of --qos-profile
#                         option)
#   --qos-history {system_default,keep_last,keep_all,unknown}
#                         History of samples setting to publish with (overrides history value of
#                         --qos-profile option, default: system_default)
#   --qos-reliability {system_default,reliable,best_effort,unknown}
#                         Quality of service reliability setting to publish with (overrides
#                         reliability value of --qos-profile option, default: system_default)
#   --qos-durability {system_default,transient_local,volatile,unknown}
#                         Quality of service durability setting to publish with (overrides
#                         durability value of --qos-profile option, default: system_default)
# ros2 topic pub -t <times> <topic_name> <message_type> <message_content>
ros2 topic pub -t 4 /turtle1/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 2.0
  y: 2.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
"
```

**Aftermath:**

![TurtleSim Screenshot](https://user-images.githubusercontent.com/123366776/216478300-b0fb1c21-45be-43ce-87f4-69f7575876b1.png)

## Create Basic ROS2 Package

A ROS2 package is like a container of ROS2 software. It can contain code for nodes, services, topics, messages… a whole ROS2 subsystem. By default, you can create a ROS2 package with two different build tools:

- Python - originates project in Python
- CMake - originates project in C++

### Create Package

To create a simple single node package:

**CMAKE**

```bash
cd [project_dir]
mkdir src
cd src
ros2 pkg create --build-type ament_cmake --node-name [desired_node_name] [desired_package_name] # CMake version
```

![Screenshot package code editor](https://user-images.githubusercontent.com/123366776/216478365-30e80f9d-96ab-407a-8ae3-d23f5cc60d13.png)

**PYTHON**

```bash
cd [project_dir]
mkdir src
cd src
ros2 pkg create --build-type ament_python --node-name [desired_node_name] [desired_package_name]
```

![Screenshot package code editor](https://user-images.githubusercontent.com/123366776/216478388-27552399-c107-4c2a-aba1-65b3a0d4d8ec.png)

### Compile

To compile any project, go to the base folder (parent folder to src)

```bash
colcon build 
```

### Run package

To run package:

```bash
ros2 run [package_name] [node_name]
```

## RQT

rqt is a Qt-based framework for GUI development for ROS. Essentially, it enables the construction of simple GUIs for ROS programs.

```bash
rqt
```

![RQT Screenshot](https://user-images.githubusercontent.com/123366776/216478431-5ed90929-9d77-4d20-b475-c35fdd4137ef.png)

You can not only construct GUIs with it, but it also serves as a GUI for ROS2 itself, as you can consult information on topics, subscribers and others without previously programming these interfaces.

# EUFS SIM - Usage

The first thing presented to the user after launch is the EUFS SIM launcher GUI:

## Launcher Menu

In this menu, the user can choose:

- **GUI** **for the Simulator** - RViz is default but Gazebo seems better to visualize events (you can use both)
- **Vehicle Model:** the physical vehicle model to be used - Dynamic Bicycle or Point Mass (more can be created)
- **Track** (pista)
- **Command Mode:** TODO: figure what this does
- **Robot Name**
- **Vehicle Model Presets** - wet track vs dry track
- Etc.

![EUFS Sim Launcher Menu](https://user-images.githubusercontent.com/123366776/216478468-8b0b09d6-39b7-49ba-b976-993f2d892b03.png)


## GUIs

### RViz

The RViz GUI has many functionalities:

- Visualize the car model up close
- Investigate properties on the model of the car (measurement and others)
- TODO: search functionalities and complete this part

![EUFS Sim Rviz](https://user-images.githubusercontent.com/123366776/216478503-83274fd3-5c03-4c02-92b8-7b400104fe2f.png)

### Gazebo

The Gazebo GUI seems a little bit more intuitive for the simulation of events. Functionalities:

- Visualize the track
- Check properties of the surrounding environment (sun position, cones position and properties, atmosphere and wind properties)
- Change camera view points and configurations
- TODO: search functionalities and complete this part

![EUFS Sim Gazebo](https://user-images.githubusercontent.com/123366776/216478614-7571817f-1250-4115-a407-1cbd747d9b8c.png)

## RQT

Together with the program, the RQT is also launched, with a GUI constructed to ease the configuration of the event/mission to be performed. It enables the user to:

- Select the Mission for the vehicle to perform through the **************************************Mission Control GUI**************************************. The mission can also be set to ‘Manual Drive’
- Control the vehicle manually through the **********************************************EUFS Robot Steering GUI**********************************************; this GUI only works when the mission set is ‘Manual Drive’
- Set the name of the topic from where the vehicle should read the steering and control information in case of an autonomous driving mission

![EUFS Sim RQT](https://user-images.githubusercontent.com/123366776/216478652-552aac3d-e18a-42d4-8473-4b915f647796.png)


The following image depicts how the state machine defining the robots state works with inputs on the RQT GUI.

![EUFS Sim State Machine Diagram](https://user-images.githubusercontent.com/123366776/216478692-545dcaef-8f5b-4de5-bb6e-6eed479799ed.png)

To better understand the simulator mission system, it is advised to read the [full wiki page](https://gitlab.com/eufs/eufs_sim/-/wikis/State-Machine) from where this picture was taken.

## Interaction Via ROS2

The simulator’s purpose is to test the system we are developing in a simulated environment closely resembling the real environment of FS-AI. Having said that, the most important part of the simulator is how to interact with it via our programs, meaning, the opposite of manual drive.

The repo provides a diagram that illustrates how the system is suppose to work with the integration of the modules to develop by our team.

![EUFS Sim Modules Diagram](https://user-images.githubusercontent.com/123366776/216478738-c9f33018-2ba5-4a73-b209-b4f8a310ee47.png)

### Using CLI - Publishing to Control

When launching the GUI, the RQT GUI will show a topic name that can be redefined. This topic is the topic from where the simulator will receive the control information to drive the car. The default topic is **/cmd**. To obtain the message format:

 

```bash
# Check the messages for /cmd topic 
ros2 topic info /cmd
# OUTPUT
# Type: ackermann_msgs/msg/AckermannDriveStamped
# Publisher count: 1
# Subscription count: 1
# Check the message format
ros2 interface proto ackermann_msgs/msg/AckermannDriveStamped
# OUTPUT
# "header:
#   stamp:
#     sec: 0
#     nanosec: 0
#   frame_id: ''
# drive:
#   steering_angle: 0.0
#   steering_angle_velocity: 0.0
#   speed: 0.0
#   acceleration: 0.0
#   jerk: 0.0
# "
```

Posterior to that, you need to certify that the simulator is in ‘****DRIVING’**** mode and in an self-driving event.

![Mission Control GUI on Driving mode in Acceleration event](https://user-images.githubusercontent.com/123366776/216478796-768582f8-d6cd-4ab8-80a2-f606b02bd8ce.png)

Mission Control GUI on Driving mode in Acceleration event

To publish a message:

```bash
# Publish once
ros2 topic pub -1 /cmd ackermann_msgs/msg/AckermannDriveStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
drive:
  steering_angle: 0.0
  steering_angle_velocity: 0.0
  speed: 0.0
  acceleration: 2.0
  jerk: 0.0
"
# Publish 5 times
ros2 topic pub -t 5 /cmd ackermann_msgs/msg/AckermannDriveStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
drive:
  steering_angle: 0.0
  steering_angle_velocity: 0.0
  speed: 0.0
  acceleration: 2.0
  jerk: 0.0
"
# /cmd topic does not support recursive publication of a message through CLI
```

Aftermath of the latter command:

![EUFS Sim interaction result](https://user-images.githubusercontent.com/123366776/216478834-eab4d9e3-d15f-4e0b-bc3b-f3d3e20e4777.png)

### Using a Node (via programming) - Publishing to Control

TODO: Complete this

### Using CLI - Reading Sensor Data

TODO: Investigate where camera data comes from

### Using a Node (via programming) - Publishing Sensor Data

# Notes

- Weird error when trying to get information on the groundtruth topics of the SIM: “The passed message type is invalid” and “The message type 'eufs_msgs/msg/ConeArrayWithCovariance' is invalid”
