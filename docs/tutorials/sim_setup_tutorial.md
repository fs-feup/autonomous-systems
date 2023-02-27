# EUFS SIM Simulator Tutorial

This tutorial is meant to guide on the setup and installation of the [EUFS Simulator](https://gitlab.com/eufs/eufs_sim) and its prerequesites.

The [official GitLab repository](https://gitlab.com/eufs/eufs_sim) already has a fairly complete guide, having only some few details unmentioned or out of date.

## Links

- [EUFS SIM repo](https://gitlab.com/eufs/eufs_sim)
- [ROS2 galactic documentation](https://docs.ros.org/en/galactic/index.html)

## Keywords

- ROS - Robot Operating System
- ROS2 - New version of ROS that will be used
- ROS and ROS2 are also used to mention the system behind both that is common to one another
- CLI - Command Line Interface
- SIM - Simulator

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

Run every time you open your shell (**important** this is meant to be run in any shell that wants to use ros2 commands involving **eufs** custom interfaces).

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

