[![Testing and Building](https://github.com/fs-feup/autonomous-systems/actions/workflows/build.yml/badge.svg)](https://github.com/fs-feup/autonomous-systems/actions/workflows/build.yml)
[![Doxygen Documentation Page](https://github.com/fs-feup/autonomous-systems/actions/workflows/doxygen.yml/badge.svg)](https://github.com/fs-feup/autonomous-systems/actions/workflows/doxygen.yml)
[![Bugs](https://sonarcloud.io/api/project_badges/measure?project=fs-feup_autonomous-systems&metric=bugs)](https://sonarcloud.io/summary/new_code?id=fs-feup_autonomous-systems)
[![Code Smells](https://sonarcloud.io/api/project_badges/measure?project=fs-feup_autonomous-systems&metric=code_smells)](https://sonarcloud.io/summary/new_code?id=fs-feup_autonomous-systems)
[![Reliability Rating](https://sonarcloud.io/api/project_badges/measure?project=fs-feup_autonomous-systems&metric=reliability_rating)](https://sonarcloud.io/summary/new_code?id=fs-feup_autonomous-systems)
[![Security Rating](https://sonarcloud.io/api/project_badges/measure?project=fs-feup_autonomous-systems&metric=security_rating)](https://sonarcloud.io/summary/new_code?id=fs-feup_autonomous-systems)
[![Maintainability Rating](https://sonarcloud.io/api/project_badges/measure?project=fs-feup_autonomous-systems&metric=sqale_rating)](https://sonarcloud.io/summary/new_code?id=fs-feup_autonomous-systems)

# FS-FEUP Autonomous Systems

This repository holds most of the data and all the code on the project for Formula Student Autonomous Systems Department. The project being developed is an autonomous driving system.

## Starting to Develop

Before starting, check out:
- [Project Rules](https://www.notion.so/FS-FEUP-HUB-6873ab8de3b44fad990d264023fbce8b?pvs=4) in Notion (specifically check Software Development rules)
- [Startup Guide](./docs/tutorials/startup_guide.md)

## Documentation
- [Documentation Home](./docs)
- [Startup Guide](./docs/tutorials/startup_guide.md)
- [Project Specification](./docs/project-specification.md)

## Contributing

This guide aims to clarify the necessary steps for creating code that will result in an accepting pull request following the project's norms.

You work with your **shell always in the root directory**. In it, you can find mutiple scripts that will help you on the upcoming tasks. You need to install the tools in [this file](./docs/project-specification.md) to contribute. The tutorials listed before this one on the [Startup Guide](./docs/tutorials/startup_guide.md) should have given you a decent insight on their purposes and how to use them in this project.

### Natural Workflow 

- Run dependencies_install.sh script to make sure everything is installed: ```./dependencies_install.sh```
- Program
- Compile (see guide at the end of this file)
  ```sh
  colcon build --packages-select [your package and others necessary]
  ```
  you can and should use bear to update the compile_commands.json file for Sonarlint to be kept up to date, especially if you have added new files
  ```sh
  bear -- colcon build --packages-select [your package and others necessary]
  ```
- Source the packages:
  ```sh
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
- Finally push changes to repository
  ```sh
  git add -A
  git commit -m "feat(<package>): <description>"
  git push
  ```
  Note that the commit message should more or less follow the [Conventional Commits norms](https://www.conventionalcommits.org/en/v1.0.0-beta.4/)
- Verify successfull workflows

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
| Perception | perception | perception | colcon build --packages-select perception custom_interfaces | ros2 run perception perception |
| All | - | - | colcon build --symlink-install | -
| Evaluation Module | plots | plots | colcon build --packages-select plots eufs_msgs custom_interfaces | ros2 run plots plots |
| Mission Control | can | can | colcon build --packages-select can custom_interfaces | ros2 run can can |

<br>
