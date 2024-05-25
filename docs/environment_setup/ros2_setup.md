# ROS2 Setup

ROS (Robot Operating System) is a tool that allows engineers to easily program robots, providing a system to integrate programs in the robot as well as very handy tools that would otherwise required implementation. ROS2 is a new version of ROS with a few improvements, but the same core idea. You can check the difference between the two version here.

Both ROS and ROS2 are organized into distributions (releases). These distributions mostly vary in the packages available for usage and their implementations. The distribution that FS FEUP will use in the second year is the ROS2-humble, as it is the most stable one and with the farthest End-Of-Life. 

This tutorial presents two different manners of setting up ROS2 humble:
1. Through Mamba Conda and Robostack - fast to perform and easy to remove but limited
2. Normal installation of ROS2 in the system.

The first one can be interesting if you want a quick setup and one that functions well with python virtual environments. However, the second one is the typically recommended one.

## Normal ROS2 Installation

There are two alternatives:
- os packages for some OSs
- source code installation and compilation (might fix some problems that the first option may have)
You can find very explicit guides for both methods in [this website](https://docs.ros.org/en/humble/Installation.html).
This chapter describes the guide for installation through packages in Ubuntu.

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
sudo apt install ros-humble-desktop
```
**Dev tools:**
```sh
sudo apt install ros-dev-tools
```
### Environment Setup

Run every time you open your shell (**important** this is meant to be run in any open shell that wants to use ros2 commands involving **eufs** custom interfaces).

```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```

Or edit your shell rc file

```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Testing Environment

Run:
```bash
source /opt/ros/humble/setup.bash
rviz2
```
If all correct, should open a window with no errors.

#### Other examples

In one terminal, source the setup file and then run a C++ `talker`:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

In another terminal source the setup file and then run a Python `listener`:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see the `talker`
 saying that it’s `Publishing`
 messages and the `listener`
 saying `I heard`
 those messages. This verifies both the C++ and Python APIs are working properly. Hooray!

Install pip3 if you haven’t

```bash
sudo apt install python3-pip
```
