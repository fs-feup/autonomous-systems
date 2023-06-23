# Getting to Know the Project

This document serves as a guide for compiling and running the project, as well as other necessary information for getting around the code base and programming. The guide aims to provide you with the necessary information to become familiar and comfortable with the project.

### Note:
Before proceeding with this guide, please ensure that you have the following prerequisites in place:

- [ROS2 and Simulator Setup](sim_setup_tutorial.md#Nonfunctional-requirements#Colcon-setup)

## Cloning the project

Before anything else, you need to clone the project into your laptop using [git](https://git-scm.com/book/en/v2/Getting-Started-About-Version-Control). Make sure you setup an ssh connection with the repository. For more details on that, checkout the Github tutorials: [Generating a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) and [Adding a new SSH key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).

## File Structure

This project is **divided into multiple folders** inside **src** folder. Each of these folders contain a ROS2 package. Essentially, the project is composed of multiple ROS2 packages whose nodes interact with each other in runtime. More details on the system's architecture can be found in [here](../project-specification.md). 

### Notes on C++ Modules

- Each module is divided into three folders:
  - **src:** executables *.cpp files
  - **include:** libraries and headers *.hpp files
  - **test:** test files
- CMakeLists.txt is used as a build system (ROS2 default): it is a tool used to generate makefiles for c++. Everytime you creat e a new .cpp file, it must be added to the list of executable files in this CMakeLists.txt

## Step By Step

1. **Clone** the project using [git](https://git-scm.com/book/en/v2/Getting-Started-About-Version-Control)

2. **Navigate** to the project's "driverless" folder and access the "src" directory.
```sh
cd driverless/src
```

3. Once you have set up the environment, it's time to **compile** the project using [Colcon](sim_setup_tutorial.md#Nonfunctional-requirements#Colcon-setup)

If you only wish to **compile specific packages**, you can use the following command:
```sh
colcon build --packages-select [your package and others necessary]
```
Alternatively, to **compile everything**, execute:
```sh
colcon build
```

As a starting point, let's try to **compile** a single package, _control_master_. Run the following command:

```sh
colcon build --packages-select control_mpc
```

If the compilation is successful, you will see a **success message**.

4. Whenever you open a new shell, you **must** run the following command:

```sh
source install/setup.bash
```

5. Now that the project is compiled, you are ready to **run** the project's nodes. We will once again use the *control_mpc* package.

To **run any node**, use the following command:

```sh
ros2 run [package_name] [node_name]
```

In our case, let's run the previously compiled package:

```sh
ros2 run control_mpc control_mpc
```

6. Now, attempt to compile all the packages and run all the nodes. Note that for the nodes to function properly, the simulator should be running in the same OS. Refer to the table below to help you identify the packages and nodes:

<br>

| Module | Package name | Node name | Other mandatory packages |
| ------ | ------------ | --------- | ------------------------ |
| Control | control | control_mpc | | 
| Localization and Mapping | loc_map | loc_map | custom_interfaces |
| Perception | | |
| Path Planning | | |

<br>

7. The mechanism associated with running the **unit and integration tests** developed for each node is similar to the compilation one, as it also uses colcon. If you only wish to **run specific packages' tests**, you can use the following command:
```sh
colcon test --packages-select [your package and others necessary]
```
Alternatively, to **run all tests**, execute:
```sh
colcon test
```

To get a **more verbose output** of the results, use 'event-handler=console_direct+' flag:

```sh
colcon test --event-handler=console_direct+
```
