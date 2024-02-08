[![Static Analysis](https://github.com/fs-feup/autonomous-systems/actions/workflows/static-analysis.yml/badge.svg)](https://github.com/fs-feup/autonomous-systems/actions/workflows/static-analysis.yml)
[![Testing and Building](https://github.com/fs-feup/autonomous-systems/actions/workflows/build.yml/badge.svg)](https://github.com/fs-feup/autonomous-systems/actions/workflows/build.yml)
[![Doxygen Documentation Page](https://github.com/fs-feup/autonomous-systems/actions/workflows/doxygen.yml/badge.svg)](https://github.com/fs-feup/autonomous-systems/actions/workflows/doxygen.yml)

# FS-FEUP Autonomous Systems

This repository holds most of the data and all the code on the project for Formula Student Autonomous Systems Department. The project being developed is an autonomous driving system.

## Starting to Develop

Before starting, check out:
- [Project Rules](https://docs.google.com/document/d/1-YuD-V7zwE_rMwYZ7jmOQysz29_Zj9U-KQ5oIUk4dAc/edit?usp=sharing) and [AS Rules](https://docs.google.com/document/d/1kmiW4-pkKHlYM9V2sTS_4IJR4ODCaD_m/edit?usp=sharing&ouid=108427086324647392265&rtpof=true&sd=true)
- [Startup Guide](./docs/tutorials/startup_guide.md)

## Documentation
- [Documentation Home](./docs)
- [Startup Guide](./docs/tutorials/startup_guide.md)
- [Project Rules](./docs/project-rules.md)
- [Project Specification](./docs/project-specification.md)

## Contributing

This guide aims to clarify the necessary steps for creating code that will result in an accepting pull request following the project's norms.

You work with your **shell always in the root directory**. In it, you can find mutiple scripts that will help you on the upcoming tasks. You need to install the tools in [this file](./docs/project-specification.md) to contribute. The tutorials listed before this one on the [Startup Guide](./docs/tutorials/startup_guide.md) should have given you a decent insight on their purposes and how to use them in this project.

### Natural Workflow 

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
- Run static analysis (runs clang-format, cpplint, cppcheck, ruff):
  ```sh
  ./static-tools.sh all
  ```
  or just checking part:
  ```sh
  ./static-tools.sh check
  ```
  or just modifying tools:
  ```sh
  ./static-tools.sh act
  ```
  or individually like so:
  ```sh
  ./static-tools.sh clang-format # C++
  ./static-tools.sh cppcheck # C++
  ./static-tools.sh cpplint # C++
  ./static-tools.sh ruff # Python
  ```
- Finally push changes to repository
  ```sh
  git add -A
  git commit -m "feat(<package>): <description>"
  git push
  ```
  Note that the commit message should more or less follow the [Conventional Commits norms](https://www.conventionalcommits.org/en/v1.0.0-beta.4/)


### Extra Notes

Whenever you open a new shell, you **must** run the following command:

```sh
source install/setup.bash
```

You can also change the **log configurations** of the running node:
```sh
ros2 run [package_name] [node_name] --ros-args --log-level [node_name]:=[log_level] # Can be warn, error, info and debug
```

#### Compilation Dependencies

<br>

| Module | Package name | Node name | Compilation command | Running command | 
| ------ | ------------ | --------- | ------------------------ | -------|
| Localization and Mapping | loc_map | loc_map | colcon build --packages-select loc_map custom_interfaces eufs_msgs fs_msgs | ros2 run loc_map loc_map | 
| Path Planning | planning | planning | colcon build --packages-select custom_interfaces planning | ros2 run planning planning |
| All | - | - | colcon build --symlink-install | -
| Evaluation Module | plots | plots | colcon build --packages-select plots eufs_msgs custom_interfaces | ros2 run plots plots |
| Mission Control | can | can | colcon build --packages-select can custom_interfaces | ros2 run can can |

<br>
