# Environment Setup

There are multiple ways to develop software for the autonomous systems' pipeline. Some environments have been configured before by team members and have been established as good solutions. The steps for setting up these environments is described in various documents that will be listed below. The documents are listed by order of preference, with the first document having the best environment overall. A member can choose not to use any of these environments, so long his setup does not stop him from doing anything that the ones listed below would allow.

Much of the software developed on this department is only compatible or preferrably compatible with Ubuntu. As such, it is highly advisable to use it, specifically Ubuntu 22.04. If you want to use another one, do it at your own risk. In any way, some of the development environment options might work decently well with a native Windows environment. Here are the options studied:

1. **Docker Environment on Ubuntu** - Having Ubuntu 22.04 installed in your disc, with VSCode, and docker and the development environment running inside a docker container.
2. **Docker Environment on Windows WSL** - Having Windows installed in your disc, with WSL (Windows Subsystem for Linux), VSCode and docker and the development environment running inside a docker container.
3. **Docker Environment on Mac** - Having a Mac with Docker desktop docker and the development environment running inside a docker container. 
4. **Docker Environment on Windows** - Having Windows installed in your disc, without WSL (Windows Subsystem for Linux), VSCode and docker and the development environment running inside a docker container.
5. **Direct installation on Ubuntu** - Having Ubuntu 22.04 installed in your disc, with VSCode, and the development environment running inside a directly on the OS.
6. **Direct installation on Windows WSL** - Having Windows installed in your disc, with WSL (Windows Subsystem for Linux), VSCode and the development environment running inside WSL.


## First Step - Install OS and Major Tools

For each of the approaches, there are different steps here:

1. **Docker Environment on Ubuntu**
    1. Install Ubuntu 22.04 - [video tutorial](https://www.youtube.com/watch?v=GXxTxBPKecQ)
    2. Install [Docker](./environment_setup/docker-install.md)
2. **Docker Environment on Windows WSL**
    1. Install Windows - if you don't have it, don't do it, Windows sucks
    2. Install WSL 2 inside windows - [tutorial](./environment_setup/installing_wsl.md)
    3. Install [Docker](./environment_setup/docker-install.md)
    4. Install [VcXSrv](https://sourceforge.net/projects/vcxsrv/) - for graphical interfaces ([example of usage](https://www.youtube.com/watch?v=BDilFZ9C9mw)) 
3. **Docker Environment on Mac** both steps can be seen being executed in [this tutorial](https://www.youtube.com/watch?v=cNDR6Z24KLM)
    1. Install [Docker](./environment_setup/docker-install.md)
    2. Install [XQuartz](https://www.xquartz.org/) - for graphical interfaces
4. **Docker Environment on Windows**
    1. Install [Docker](./environment_setup/docker-install.md)
    2. Install [VcXSrv](https://sourceforge.net/projects/vcxsrv/) - for graphical interfaces ([example of usage](https://www.youtube.com/watch?v=BDilFZ9C9mw)) 
5. **Direct installation on Ubuntu**
    1. Install Ubuntu 22.04 - [video tutorial](https://www.youtube.com/watch?v=GXxTxBPKecQ)
6. **Direct installation on Windows WSL**
    1. Install Windows - if you don't have it, don't do it, Windows sucks
    2. Install WSL 2 inside windows - [tutorial](./environment_setup/installing_wsl.md)

## Second Step - Set Up Coding Environment

The second step is common to all approaches: install vscode and clone this repository. Follow the [coding_environment](./environment_setup/coding_environment.md) tutorial for this.

## Third Step - Install Project Dependencies and Libraries

For each of the approaches, there are different steps here:

1. **Docker Environment** - Follow [this tutorial](./environment_setup/ros2_docker_vscode_coding_environment.md)
2. **Direct installation** - Follow [this tutorial](./environment_setup/ros2_setup.md)

## Tools

### Submodules

This project depends on submodules to function. Most, if not all of these submodules are other repositories of this organization. 
- For all environments:
    ```sh
    git submodule update --init --recursive ./ext/interfaces
    ```
- For each simulator:
    ```sh
    git submodule update --init --recursive ./ext/pacsim # Pacsim
    git submodule update --init --recursive ./ext/amz-fssim # AMZ Racing SIM
    git submodule update --init --recursive ./ext/eufs-sim # Edinburgh FS SIM 
    ```
- For in vehicle testing:
    ```sh
    git submodule update --init --recursive ./ext/rslidar_sdk # LiDAR
    git submodule update --init --recursive ./ext/rslidar_msg # LiDAR
    git submodule update --init ./ext/as-integration # ROS_CAN, NOT RECURSIVE
    ```

Check the [git advanced tutorial](./git_advanced.md) for more information.

### Simulator Setup

We use multiple simulators in our project:

#### FSDS

The simulator requires Ubuntu to work (or WSL). The tutorial regarding its setup is [here](./environment_setup/fsds_setup.md).

#### PacSim

This simulator is very light and can run on anything, inside or outside a docker container. Follow the [tutorial](./environment_setup/pacsim_setup.md), or repher to its [fork repo](https://github.com/fs-feup/pacsim) right away.

### Submodules

This project depends on submodules to function. Most, if not all of these submodules are other repositories of this organization. 
- For all environments:
    ```sh
    git submodule update --init --recursive ./ext/interfaces
    ```
- For each simulator:
    ```sh
    git submodule update --init --recursive ./ext/pacsim # Pacsim
    git submodule update --init --recursive ./ext/amz-fssim # AMZ Racing SIM
    git submodule update --init --recursive ./ext/eufs-sim # Edinburgh FS SIM 
    ```
- For in vehicle testing:
    ```sh
    git submodule update --init --recursive ./ext/rslidar_sdk # LiDAR
    git submodule update --init --recursive ./ext/rslidar_msg # LiDAR
    git submodule update --init ./ext/as-integration # ROS_CAN, NOT RECURSIVE
    ```

Check the [git advanced tutorial](./git_advanced.md) for more information.


### Static Analysis

Some other tools are required for development in the AS department, such as Static Analysis tools. 
If you are not using the docker dev environment, make sure you configure them for your environment.
