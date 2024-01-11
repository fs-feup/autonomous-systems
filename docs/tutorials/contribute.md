# Contributing

This guide aims to clarify the necessary steps for creating code that will result in an accepting pull request following the project's norms.

You work with your **shell always in the root directory**. In it, you can find mutiple scripts that will help you on the upcoming tasks. You need to install the tools in [this file](../project-specification.md) to contribute. The tutorials listed before this one on the [Startup Guide](./startup_guide.md) should have given you a decent insight on their purposes and how to use them in this project.

## Natural Workflow 

- Run dependencies_install.sh script to make sure everything is installed: ```./dependencies_install.sh```
- Program
- Compile (see guide at the end of this file)
  ```sh
  colcon build --packages-select [your package and others necessary] --symlink-install
  source install/setup.bash
  ```
- Run code (`ros-args` are optional) (see guide at the end of this file)
  ```sh
  ros2 run [package_name] [node_name] (--ros-args -p [param_name]:=[param_value])
  ```
- Run tests
  ```sh
  colcon test --packages-select [your package and others necessary] [--event-handler=console_direct+] #last part for verbose
  ```
- Run static analysis (runs clang-format, cpplint, cppcheck, ruff and doxygen):
  ```sh
  ./static-tools.sh all
  ```
  or individually like so:
  ```sh
  ./static-tools.sh clang-format # C++
  ./static-tools.sh cppcheck # C++
  ./static-tools.sh cpplint # C++
  ./static-tools.sh ruff # Python
  ./static-tools.sh doxygen # Both
  ```
- Finally push changes to repository
  ```sh
  git add -A
  git commit -m "feat(<package>): <description>"
  git push
  ```
  Note that the commit message should more or less follow the [Conventional Commits norms](https://www.conventionalcommits.org/en/v1.0.0-beta.4/)


## Notes on Nodes, Packages, Compiling, Executing and Dependencies

### Compile and Run Extended

If you only wish to **compile specific packages**, you can use the following command:
```sh
colcon build --packages-select [your package and others necessary]
```
Alternatively, to **compile everything**, execute:
```sh
colcon build --symlink--install # Extra required for perception
```

If the compilation is successful, you will see a **success message**.

Whenever you open a new shell, you **must** run the following command:

```sh
source install/setup.bash
```

```sh
ros2 run [package_name] [node_name]
```


You can also change the **log configurations** of the running node:
```sh
ros2 run [package_name] [node_name] --ros-args --log-level [node_name]:=[log_level] # Can be warn, error, info and debug
```

### Compilation Dependencies

<br>

| Module | Package name | Node name | Compilation command | Running command | 
| ------ | ------------ | --------- | ------------------------ | -------|
| Localization and Mapping | loc_map | loc_map | colcon build --packages-select loc_map custom_interfaces eufs_msgs fs_msgs | ros2 run loc_map loc_map | 
| Path Planning | planning | planning | colcon build --packages-select custom_interfaces planning | ros2 run planning planning |
| All | - | - | colcon build --symlink-install | -
| Evaluation Module | plots | plots | colcon build --packages-select plots eufs_msgs custom_interfaces | ros2 run plots plots |
| Mission Control | can | can | colcon build --packages-select can custom_interfaces | ros2 run can can |

<br>

### Testing

The mechanism associated with running the **unit and integration tests** developed for each node is similar to the compilation one, as it also uses colcon. If you only wish to **run specific packages' tests**, you can use the following command:
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
