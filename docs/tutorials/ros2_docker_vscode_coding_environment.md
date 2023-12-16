# ROS2 + Docker + VSCode Coding Environment

## Requirements
- Install VSCode: https://code.visualstudio.com/
- Install Docker (Engine or Desktop): https://docs.docker.com/engine/
## Installation Steps
Inside VSCODE, install Remote Development extension
### Workspace Setup
Create a workspace, or if you have your own, skip this step.
```bash
cd ~/
mkdir ws_[project]
cd ws_[project]
mkdir src
```
### Docker Setup
Now create a .devcontainer folder in the root of your workspace and add a devcontainer.json and Dockerfile to this .devcontainer folder. Additionally, you need to create a cache folder in which you can cache the build and install folders for different ROS 2 distros. Make sure to create the distro, build, log, and install folders.
```ssh
ws_[project]
├── cache
|   ├── [ROS2_DISTRO]
|   |   ├── build
|   |   ├── install
|   |   └── log
|   └── ...
|
├── src
    ├── .devcontainer
    │   ├── devcontainer.json
    │   └── Dockerfile
    ├── package1
    └── package2
```
Go to VSCode, File->Open Folder... src/ folder of the workspace (not the root).

For the Dev Container to function properly, we have to build it with the correct user. Therefore add the following to .devcontainer/devcontainer.json
```json
{
    "name": "ROS 2 Development Container",
    "privileged": true,
    "remoteUser": "user",
    "build": {
        "dockerfile": "Dockerfile",
        "args": {
            "USERNAME": "user"
        }
    },
    "workspaceFolder": "/home/ws",
    "workspaceMount": "source=${localWorkspaceFolder},target=/home/ws/src,type=bind",
    "customizations": {
        "vscode": {
            "extensions":[
                "ms-vscode.cpptools",
                "ms-vscode.cpptools-themes",
                "twxs.cmake",
                "donjayamanne.python-extension-pack",
                "eamodio.gitlens",
                "ms-iot.vscode-ros"
            ]
        }
    },
    "containerEnv": {
        "DISPLAY": "unix:0",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "ROS_DOMAIN_ID": "42"
    },
    "runArgs": [
        "--net=host",
        "-e", "DISPLAY=${env:DISPLAY}"
    ],
    "mounts": [
       "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached",
        "source=/dev/dri,target=/dev/dri,type=bind,consistency=cached",
        "source=${localWorkspaceFolder}/../cache/ROS_DISTRO/build,target=/home/ws/build,type=bind",
        "source=${localWorkspaceFolder}/../cache/ROS_DISTRO/install,target=/home/ws/install,type=bind",
        "source=${localWorkspaceFolder}/../cache/ROS_DISTRO/log,target=/home/ws/log,type=bind"
    ],
    "postCreateCommand": "sudo rosdep update && sudo rosdep install --from-paths src --ignore-src -y && sudo chown -R user /home/ws/"
}
```
With CTRL+F find "user" and replace it by your pc user (for sudo access). Run `echo $USERNAME` in the terminal if you don't know this.
Find "ROS_DISTRO" and replace it by your distro (in this case 'humble').
8:
Add the following to the Dockerfile
```dockerfile
FROM ros:ROS_DISTRO
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
```

Again replace "USERNAME" by your user and ROS_DISTRO by your ROS2 distro.
### Attaching VSCode to Dev Container
Do CTRL+SHIFT+P and select `Dev Containers: (Re-)build and Reopen in Container` and execute.

Open terminal on host and run `xhost +`.
### Testing Environment
Run:
```bash
sudo apt install ros-$ROS_DISTRO-rviz2 -y
source /opt/ros/$ROS_DISTRO/setup.bash
rviz2
```
If all correct, should open a window with no errors.
