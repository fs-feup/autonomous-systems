# Formula Student Driverless Simulator

The Formula Student Driverless Simulator is a simulator originally created for online competitions for formula student driverless autonomous pipelines. It is well maintained, as far as FS simulators concern (it has some incoherences in documentation). This tutorial will help you set up the simulator in your computer and configure the system to be able to communicate with it through ROS, even with the AS pipeline running inside a container.

## Installation Steps

1. First and unfortunately, you will have to install ROS2 in your system. Follow the tutorial [here](./environment_setup/ros2_setup.md) or in the [official webpage](https://docs.ros.org/en/humble/Installation.html).
2. Then you have to clone the repository into your ubuntu's home folder, otherwise it will not work. You can clone the latest stable version like so:
    ```sh
    git clone --depth 1 --branch v2.2.0 git@github.com:FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules
    ```
    The submodules installation will most likely fail and, as such, you will have do download them by hand. These are the ros2 fs_msgs repository and rpclib, which have to be cloned into the 'ros2/src' and 'AirSim/external' folders respectively. A zip file with the repo and these two submodules already in the correct location can be found in the team's drive [here](https://drive.google.com/file/d/13XHk4i1tjqSn7-eSarUmbNudpPpa1mfx/view?usp=drive_link), meaning you can download this folder instead of doing the steps mentioned above.
3. Build the AirSim library (from the repo's home folder):
    ```sh
    ./AirSim/setup.sh
    ./AirSim/build.sh
    ```
4. Build the ROS2 bridge package:
    ```sh
    cd ros2
    source /opt/ros/iron/setup.bash
    colcon build
    ```
5. Download the simulator's binaries (yes, I know, idotic when you have just downloaded the sources but that is what we got if you don't want to spend a day compiling UnrealEngine). Download them to the same version. Link [here](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/tag/v2.2.0). Authorize executing permissions for the script in the root folder like so ```chmod u+x FSDS.sh```.


## Running the Simulator

You are going to need 2 shells for this (or leave the simulator running on background, which is not advised as there aren't many clear ways to shut it down other than Ctrl+C). On the first shell:
1. Travel to the sim's binaries directory
2. Run the simulation: 
    ```sh
    ./FSDS.sh
    ```
    . This will take up the whole screen, use windows / pane switching commands to switch between this and the terminal or another monitor, something like that.
3. Start the simulation (with whatever configurations you see fit)

On the other shell:
1. Go the the simulator's sources (the repository you cloned first) and to the ros2 folder: 
    ```sh
    cd Formula-Student-Driverless-Simulator/ros2
    ```
2. Source ros and the pacakge:
    ```sh
    source /opt/ros/iron/setup.bash
    source ./install/setup.bash
    ```
3. Launch the package: 
    ```sh
    ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py
    ```
    You can check if it worked by running ros2 topic list and checking the topics being published.

### Connecting to the docker container

To connect the ros2 topics in your pc to the ones inside the container, in our project, make sure you have your **.devcontainer** and **Dockerfile** files updated according to the tutorial (rebuild the container if necessary). Then, just run the following command in the shell you will be running the bridge:
```sh
export ROS_DOMAIN_ID=42
```
This will ensure that the domain ids are the same inside and outside the docker container. You can add this to your shell start-up script if you wish, so that you don't have to run this on every shell every time.
