# Testing Tutorial

In this project, it is very important that all code is thoroughly and constantly tested for its functionality. We will do this through the use of unit, integration and accpetance tests as:
- we are dealing with a critical system that does not tolerate bugs
- automated tests allow for quick procedures to increase confidence in changes to the code base without requiring reviews

## Links

- [ROS2 QA page](http://wiki.ros.org/Quality/Tutorials/UnitTesting)

## Unit Testing Tools

In ROS2, the main testing tools are [unittest](https://docs.python.org/3/library/unittest.html) for python packages and [gtest](https://github.com/google/googletest) for c++ packages. Both of these tools will allow the developer to create unit and integration tests. 

## Writing Unit Tests

Unit tests aim to acess the functionality of a single function, class or code entity. This is the environment the tools mentioned above were born to thrive on. They are equiped with multiple mocking tools for the developer to be able to isolate components and test them individually. There are also many tutorials for these tools. You can also check the [ROS documentation](http://wiki.ros.org/Quality/Tutorials/UnitTesting) on these tools for more information.

To run normal unit tests, simply write ```colcon test``` in the terminal (or ```colcon test --event-handler=console_direct+ # Last part for verbose```) and hit enter (after building and sourcing the project, of course).

These two projects are setup for testing [python](../assets/test_tutorial/theman/) and [c++](../assets/environment_setup_tutorial/training/experiment1/) respectively. Check them out for reference.

## Writing Integration Tests

Integration tests acess the system's different functions from top to bottom. In our case, integration tests would reflect in something like testing the behaviour of a certain node or group of nodes.

Using the same tools as for the unit tests, we can create integration tests by simply coding other ROS2 nodes inside the tests that are able to communicate with the actual nodes of the system. This way can be seen being aplied in [this test file](../../src/localization-mapping/loc_map/test/). The idea is simply coding a simpler node to test the other nodes and launch it together with the real nodes in a mocked environment.

**Rostest** is a ROS package that allows the user to run testing nodes to ease the running of integration tests built with the tools mentioned above. Rostest is designed to integrate with gtest and unittest rather smoothely. More details on this tool can be found [here](http://wiki.ros.org/rostest).

## Further Instruction

- [ROS2 C++ Testing Tutorial Video](https://www.youtube.com/watch?v=t2Jm1Nt49-A&t=2031s)
- [ROS2 Python Testing Tutorial Video](https://www.youtube.com/watch?v=h-1IhC01T1c)
- [gtest](https://google.github.io/googletest/)
- [unittest](https://docs.python.org/3/library/unittest.html)