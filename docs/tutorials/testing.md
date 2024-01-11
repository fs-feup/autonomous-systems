# Testing Tutorial

In this project, it is very important that all code is thoroughly and constantly tested for its functionality. We will do this through the use of unit, integration and accpetance tests as:
- we are dealing with a critical system that does not tolerate bugs
- automated tests allow for quick procedures to increase confidence in changes to the code base without requiring reviews

## Links

- [ROS2 QA page](http://wiki.ros.org/Quality/Tutorials/UnitTesting)

## Unit Testing Tools

In ROS2, the main testing tools are [unittest](https://docs.python.org/3/library/unittest.html) for python packages and [gtest](https://github.com/google/googletest) for c++ packages. Both of these tools will allow the developer to create unit and integration tests. 

## Unit Testing

Unit testing is a software testing method where individual components or units of a software application are tested in isolation. The primary goal of unit testing is to validate that each unit of the software performs as expected. Typically, a unit is the smallest testable part of an application, such as a function, method, or procedure. Unit testing is often conducted by developers during the development phase to identify and fix bugs early in the development cycle, ensuring the reliability and stability of the software. This is the environment the tools mentioned above were born to thrive on. They are equiped with multiple mocking tools for the developer to be able to isolate components and test them individually. 

There are also many tutorials for these tools. You can also check the [ROS documentation](http://wiki.ros.org/Quality/Tutorials/UnitTesting) on these tools for more information.

### Structure

#### AAA structure

You can use the AAA structure to write unit tests:

1. **Arrange** – configure the test by setting up the tested system and other mechanisms.
2. **Act** – call an action to perform to test the unit.
3. **Assert** – check the result of the performed operation to verify it worked as intended.

#### One Scenario per Test

A test must only test a single scenario. Testing each program part with a single scenario at a time helps identify the exact issue when a test fails.

#### Folder structure

For better understandability, tests regarding code in a file should have a 'mirror' test file in the test folder. This way, the test and src folder have the same structure, apart from the test folder not having files that do not contain tests. This structure makes it easier to track down the relevant tests and link them to the code they test.

### How to do Unit Tests

#### Unit Tests Repeatable and Scalable

To ensure an effective testing strategy, prioritize repeatability and scalability in unit tests. Encourage synchronized unit test writing alongside application code development, adopting practices like behavior-driven or test-driven programming. Emphasize combined code and test reviews to refine testing practices, even for bug fixes. Maintain a zero-tolerance policy toward test failures, addressing issues promptly to prevent time wastage and ensure robust, reliable code deployment.

#### Mocking and Stubbing

Employ techniques such as mocking and stubbing to isolate the unit under test from its dependencies. By simulating the behavior of external components or services, developers can focus solely on testing the specific functionality of the unit. This approach enhances the reliability of unit tests and enables developers to identify and address potential issues within the isolated context.

1. **Mock:** In software testing, a mock is a simulated object that mimics the behavior of real objects in controlled ways, enabling the testing of a unit's interactions with its dependencies. Mocks are used to verify how a unit interacts with other units without invoking their actual implementations.

2. **Stub:** A stub is a minimal, simplified version of a module or object with predetermined behavior that is used in place of a fully functional component during testing. Stubs provide canned responses to method calls, allowing developers to isolate the unit under test from its dependencies during the testing process.

### Policy Objective

Every time a code change is made within the software development lifecycle, it is mandatory to execute the relevant unit tests to verify that previous functionalities remain intact and unaffected.

## Writing Integration Tests

Integration tests acess the system's different functions from top to bottom. In our case, integration tests would reflect in something like testing the behaviour of a certain node.

Using the same tools as for the unit tests, we can create integration tests by simply coding other ROS2 nodes inside the tests that are able to communicate with the actual nodes of the system (something like testing nodes). This way can be seen being aplied in [this test file](../../src/loc_map/test/integration_test.cpp). The idea is simply coding a simpler node to test the other nodes and launch it together with the real nodes in a mocked environment.

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
    colcon test --packages-select loc_map
    ```
- run only one file in C++ (Gtest)
    ```sh
    ros2 run loc_map loc_map_test # (for loc_map)
    ```
- run only one test in python ```python <test_file>```

## Further Instruction

You should watch at least some of these videos to understand the basic syntax of the tools.

- [ROS2 C++ Testing Tutorial Video](https://www.youtube.com/watch?v=t2Jm1Nt49-A&t=2031s)
- [ROS2 Python Testing Tutorial Video](https://www.youtube.com/watch?v=h-1IhC01T1c)
- [Google test tutorial](https://www.youtube.com/watch?v=JJqRlSTQlh4&t=92s)
- [gtest](https://google.github.io/googletest/)
- [unittest](https://docs.python.org/3/library/unittest.html)