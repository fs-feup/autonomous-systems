# Compile, Test, Run

This document serves as a guide for compiling and running the project, as well as other necessary information for getting around the code base and programming. The guide aims to provide you with the necessary information to become familiar and comfortable with the project.

## Step By Step

1. **Clone** the project using [git](https://git-scm.com/book/en/v2/Getting-Started-About-Version-Control)

2. **Navigate** to the project's "driverless" folder and access the "src" directory.
```sh
cd driverless/src
```

3. **Intall** all dependencies:
```sh
./install_dependencies.sh
```

4. Once you have set up the environment, it's time to **compile** the project using Colcon

If you only wish to **compile specific packages**, you can use the following command:
```sh
colcon build --packages-select [your package and others necessary]
```
Alternatively, to **compile everything**, execute:
```sh
colcon build --symlink--install # Extra required for perception
```

As a starting point, let's try to **compile** a single package, _control_master_. Run the following command:

```sh
colcon build --packages-select control_mpc
```

If the compilation is successful, you will see a **success message**.

5. Whenever you open a new shell, you **must** run the following command:

```sh
source install/setup.bash
```

6. Now that the project is compiled, you are ready to **run** the project's nodes. We will once again use the *control_mpc* package.

To **run any node**, use the following command:

```sh
ros2 run [package_name] [node_name]
```

In our case, let's run the previously compiled package:

```sh
ros2 run control_mpc control_mpc
```

You can also change the **log configurations** of the running node:
```sh
ros2 run [package_name] [node_name] --ros-args --log-level [node_name]:=[log_level] # Can be warn, error, info and debug
```


7. Now, attempt to compile all the packages and run all the nodes. Note that for the nodes to function properly, the simulator should be running in the same OS. Refer to the table below to help you identify the packages and nodes:

<br>

| Module | Package name | Node name | Compilation command | Running command | 
| ------ | ------------ | --------- | ------------------------ | -------|
| Control | control | control | colcon build --packages-select custom_interfaces eufs_msgs fs_msgs control | ros2 run control control | 
| Localization and Mapping | loc_map | loc_map | colcon build --packages-select loc_map custom_interfaces eufs_msgs fs_msgs | ros2 run loc_map loc_map | 
| Perception | perception | perception | colcon build --symlink-install --packages-select perception custom_interfaces | ros2 run perception perception | 
| Path Planning | planning | planning | colcon build --packages-select custom_interfaces planning | ros2 run planning planning |
| All 4 | - | - | colcon build --symlink-install | ros2 launch fsfeup_launcher.launch.py 
| Evaluation Module | plots | plots | colcon build --packages-select plots eufs_msgs custom_interfaces | ros2 run plots plots |
| Mission Control | can | can | colcon build --packages-select can custom_interfaces | ros2 run can can |

<br>

8. The mechanism associated with running the **unit and integration tests** developed for each node is similar to the compilation one, as it also uses colcon. If you only wish to **run specific packages' tests**, you can use the following command:
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
