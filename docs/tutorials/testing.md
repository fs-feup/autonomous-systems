# Testing Tutorial

In this project, it is very important that all code is thoroughly and constantly tested for its functionality. We will do this through the use of unit, integration and acceptance tests as:
- we are dealing with a critical system that does not tolerate bugs
- automated tests allow for quick procedures to increase confidence in changes to the code base without requiring reviews

This is a short guide focused on ROS testing. For the full rules, check the [notion rules on software testing](https://www.notion.so/Software-Testing-d891b701d9184a3ebf46824666c6d61b?pvs=4).

## Links

- [ROS2 QA page](http://wiki.ros.org/Quality/Tutorials/UnitTesting)

## Unit Testing Tools

In ROS2, the main testing tools are [unittest](https://docs.python.org/3/library/unittest.html) for python packages and [gtest](https://github.com/google/googletest) for c++ packages. Both of these tools will allow the developer to create unit and integration tests. 

## Unit Testing

Unit testing is a software testing method where individual components or units of a software application are tested in isolation. The primary goal of unit testing is to validate that each unit of the software performs as expected. Typically, a unit is the smallest testable part of an application, such as a function, method, or procedure. Unit testing is often conducted by developers during the development phase to identify and fix bugs early in the development cycle, ensuring the reliability and stability of the software. This is the environment the tools mentioned above were born to thrive on. They are equiped with multiple mocking tools for the developer to be able to isolate components and test them individually. 

There are also many tutorials for these tools. You can also check the [ROS documentation](http://wiki.ros.org/Quality/Tutorials/UnitTesting) on these tools for more information.

## Writing Integration Tests

Integration tests acess the system's different functions from top to bottom. In our case, integration tests would reflect in something like testing the behaviour of a certain node.

Using the same tools as for the unit tests, we can create integration tests by simply coding other ROS2 nodes inside the tests that are able to communicate with the actual nodes of the system (something like testing nodes). This way can be seen being aplied in [this test file](../../src/ekf_state_est/test/integration_test.cpp). The idea is simply coding a simpler node to test the other nodes and launch it together with the real nodes in a mocked environment.

Integration tests should be created for every node and possibly for a few big components.

## Continuous Integration and Continuous Deployment (CI/CD)

Integrate unit testing into the CI/CD pipeline to automate the testing process. Configure automated builds and tests that run whenever changes are made to the codebase. Implementing this practice facilitates the early detection of errors, reduces the likelihood of introducing new bugs, and streamlines the development and deployment workflow. In this project, CI-CD is performed using github actions. The related files are in [.github folder](../../.github/workflows/).

## Running the tests

To run normal tests:
- normal run
    ```sh
    colcon test
    ``` 
- check results
    ```sh
    colcon test-result --all --verbose
    ```
- direct console output
    ```sh
    colcon test --event-handler=console_direct+
    ```
- use --packages-select flag to select only one package
    ```sh
    colcon test --packages-select ekf_state_est
    ```
- run only one file in C++ (Gtest)
    ```sh
    ros2 run ekf_state_est ekf_state_est_test # (for ekf_state_est)
    ```
- run only one test in python ```python <test_file>```

## Further Instruction

You should watch at least some of these videos to understand the basic syntax of the tools.

- [ROS2 C++ Testing Tutorial Video](https://www.youtube.com/watch?v=t2Jm1Nt49-A&t=2031s)
- [ROS2 Python Testing Tutorial Video](https://www.youtube.com/watch?v=h-1IhC01T1c)
- [Google test tutorial](https://www.youtube.com/watch?v=JJqRlSTQlh4&t=92s)
- [gtest](https://google.github.io/googletest/)
- [unittest](https://docs.python.org/3/library/unittest.html)
