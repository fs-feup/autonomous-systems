# Contributing

This guide aims to clarify the necessary steps for creating code that will result in an accepting pull request following the project's norms.

You work with your **shell always in the root directory**. In it, you can find mutiple scripts that will help you on the upcoming tasks. You need to install the tools in [this file](../docs/technologies.md) to contribute. The tutorials listed before this one on the [Startup Guide](../docs/tutorials/startup_guide.md) should have given you a decent insight on their purposes and how to use them in this project.

## Natural Workflow 

- Program
- Compile
  ```sh
  colcon build --packages-select [your package and others necessary]
  source install/setup.bash
  ```
- Run code (`ros-args` are optional) (check specific instructions for each node in [here](../docs/tutorials/compile-test-run.md))
  ```sh
  ros2 run [package_name] [node_name] (--ros-args -p [param_name]:=[param_value])
  ```
- Run tests
  ```sh
  colcon test --packages-select [your package and others necessary] [--event-handler=console_direct+] #last part for verbose
  ```
- Check test results
  ```sh
  colcon test-result --all --verbose
  ```

For C++:
- Run cppcheck
  ```sh
  ./static-tools.sh cppcheck
  ```
- Format code
  ```sh
  ./static-tools.sh clang-format
  ```
- Run linter
  ```sh 
  ./static-tools.sh cpplint
  ```
For Python:
- Run liner/formatter
  ```sh
  ./static-tools.sh ruff 
  ```
For everybody:
- Run documentator
  ```sh
  ./static-tools.sh doxygen
  ```
- Finally push changes to repository
  ```sh
  git add -A
  git commit -m "feat(<package>): <description>"
  git push
  ```
  Note that the commit message should more or less follow the [Conventional Commits norms](https://www.conventionalcommits.org/en/v1.0.0-beta.4/)
