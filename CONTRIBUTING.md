# Contributing

This guide aims to clarify the necessary steps for creating code that will result in an accepting pull request following the project's norms.

You work with your **shell always in the root directory**. In it, you can find mutiple scripts that will help you on the upcoming tasks. You need to follow the tutorials listed before this one on the [Startup Guide](https://github.com/fs-feup/tutorials/blob/main/tutorials/startup_guide_as.md).

## Natural Workflow 

- Set up development environment.
- Run dependencies_install.sh script to make sure everything is installed: ```./dependencies_install.sh```
- Pull updates
  ```sh
  git pull
  ```
- Checkout new branch.
  - for new branches:
    ```sh
    git checkout -b <branch_name> # for new branhces
    ```
    or
    ```sh
    git branch <branch_name>
    git checkout <branch_name>
    ```
  - for existing branches:
    ```sh
    git checkout <branch_name>
    ```
- Program
- Compile (see guide at the end of this file)
  ```sh
  colcon build
  ```
  you can and should use bear to update the compile_commands.json file for Sonarlint to be kept up to date, especially if you have added new files
  ```sh
  bear -- colcon build 
  ```
- Source the packages:
  ```sh
  source install/setup.bash
  ```
- Run code (`ros-args` are optional) (see guide at the end of this file)
  ```sh
  ros2 run [package_name] [node_name] (--ros-args -p [param_name]:=[param_value])
  ```
  Alternatively, packages contain launch files, which can be used for running multiple nodes at a time or running a node with preset parameters:
  ```sh
  ros2 launch [package_name] [launch_file_name]
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
- Create a Pull Request to the main branch, assigning yourself and selecting reviewers
- Verify successfull workflows


## Notes on Nodes, Packages, Compiling, Executing and Dependencies

### Compile and Run Extended

If you only wish to **compile specific packages**, you can use the following command:
```sh
colcon build --packages-select [your package and others necessary]
```
Alternatively, to **compile everything**, execute:
```sh
colcon build
```

If the compilation is successful, you will see a **success message**.

Whenever you open a new shell, you **must** run the following command:

```sh
source install/setup.bash
```

This tells the system where the executables for your project are. If you did not add it to your .bashrc or .zshrc file, you also have to source ros as well: ```source /opt/ros/humblr/setup.bash```.

```sh
ros2 run [package_name] [node_name]
```

You can also change the **log configurations** of the running node:
```sh
ros2 run [package_name] [node_name] --ros-args --log-level [node_name]:=[log_level] # Can be warn, error, info and debug
```

### Testing

The mechanism associated with running the **unit and integration tests** developed for each node is similar to the compilation one, as it also uses colcon. If you only wish to **run specific packages' tests**, you can use the following command:
```sh
colcon test --packages-select [your package and others necessary]
```
Alternatively, to **run all tests**, execute:
```sh
colcon test
```

To obtain the test results:
```sh
colcon test-result
```

To get a **more verbose output** of the results, use 'event-handler=console_direct+' flag:

```sh
colcon test --event-handler=console_direct+
```

Instead, you can check the logs for the information on the tests (or build folder).
