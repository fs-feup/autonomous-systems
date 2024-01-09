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
    2. Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) (docker engine preferrably, but you can also try Docker Desktop at your own risk) - follow the apt repository method
2. **Docker Environment on Windows WSL**
    1. Install Windows - if you don't have it, don't do it, Windows sucks
    2. Install WSL 2 inside windows - [tutorial](./environment_setup/installing_wsl.md)
    3. Install [Docker Desktop on windows](https://docs.docker.com/desktop/install/windows-install/) - follow indications for WLS2 backend instead of HyperV backend
    4. Install [VcXSrv](https://sourceforge.net/projects/vcxsrv/) - for graphical interfaces ([example of usage](https://www.youtube.com/watch?v=BDilFZ9C9mw)) 
3. **Docker Environment on Mac** both steps can be seen being executed in [this tutorial](https://www.youtube.com/watch?v=cNDR6Z24KLM)
    1. Install [Docker Desktop on Mac](https://docs.docker.com/desktop/install/mac-install/)
    2. Install [XQuartz](https://www.xquartz.org/) - for graphical interfaces
4. **Docker Environment on Windows**
    1. Install [Docker Desktop on Windows](https://docs.docker.com/desktop/install/windows-install/) - - follow indications for HyperV backend
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

1. **Docker Environment on Ubuntu** - Follow [this tutorial](./environment_setup/ros2_docker_vscode_coding_environment.md)
2. **Docker Environment on Windows WSL** - Follow [this tutorial](./environment_setup/ros2_docker_vscode_coding_environment.md)
3. **Docker Environment on Mac** - Follow [this tutorial](./environment_setup/ros2_docker_vscode_coding_environment.md)
4. **Docker Environment on Windows** - Follow [this tutorial](./environment_setup/ros2_docker_vscode_coding_environment.md)
5. **Direct installation on Ubuntu** - Follow [this tutorial](./environment_setup/ros2_setup.md)
6. **Direct installation on Windows WSL** - Follow [this tutorial](./environment_setup/ros2_setup.md)
