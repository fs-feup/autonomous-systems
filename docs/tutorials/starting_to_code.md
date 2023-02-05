# Starting to code

This tutorial aims to aid in teaching the developer how to use multiple tools established for this project, as well as creating a base ROS package.

## Links

- [SSH to Virtual Machine](https://averagelinuxuser.com/ssh-into-virtualbox/)
- [VSCode](https://code.visualstudio.com/Download)

## IDE

In order to properly contribute to this project, a code editor or IDE is suggested. In this tutorial, a suggestion for an environment will be presented.

Visual Studio Code is a general purpose IDE very widely used. VSCode is our choice due to the great quantity of extensions available, namely:
- extensions for ROS environment and ROS2 syntax
- extensions for C++ and Python syntax check
- extensions for remote development via ssh (in Virtual Machines, for instance)

As the simulator required specific versions of Ubuntu to function, the setup chosen takes into account developping in the host machine while the project is held in the virtual machine.

### Steps

- Install [VSCode](https://code.visualstudio.com/Download) on HostMachine
- [Install SSH server in virtual machine](https://averagelinuxuser.com/ssh-into-virtualbox/)
- Install vscode extensions in host machine:
    - [SSH Connection 1](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh)
    - [SSH Connection 2](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh-edit)
    - [SSH Connection 3](https://marketplace.visualstudio.com/items?itemName=ms-vscode.remote-explorer)
- (Optional) Install VSCode and Extensions in Virtual Machine
    - CMake extension 
    - C++ Intellisense extension

## Doxygen

In order to help future developers start contributing to this project, code should be easy to read and staightforward. However, this is easier said than done. For this reason, it is helpful to use a documentation tool to document the code developed. Doxygen is the de-fact tool for documentation in C++. To use it you should:
- [Install doxygen](https://www.doxygen.nl/download.html) in your machine (virtual machine in this case)
    - Download
    - Follow the compilation and installation instructions in the manual
- Create doxygen file in the project home directory
    - Run ```doxygen -g``` on the command line
    - Edit the file to your liking
        - Check the INPUT variable, for this is the one that defines what files will be read to generate the documentation
- Comment the code with doxygen comments
    - Use [this vscode extension](https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen) for automatic doxygen documentation generation
- Run ```doxygen``` on the root directory to generate documentation

There is an example doxyfile [here](../assets/Doxyfile.example)

## CPPCheck

In this project, we will be using CPPCheck as a static analysis tool, to check for any underlying bugs. We will be using it in the Gitlab pipeline, but it is handy that the developer can use it locally. To install it, in the Virtual Machine, just run ```sudo apt install cppcheck```
To use it, run ```cppcheck --enable=all  <directory>```.

## File Strucure and Guidelines

A package in ROS2 can be created via the command line. The created project already constructs a certain file structure. However, some changes and additions ought to be mande in order for it to meat the guidelines established. To create a project:
1. Run ```ros2 pkg create --build-type "ament_cmake" --node-name "<node_name>" "<package_name>"
2. Add Doxyfile to the root directory (can be inside or outside the folder created, I would advise outside)
3. Create test folder inside the folder created (beside src folder)
4. Change the CMakeLists.txt to file to support the tests (change 'if(BUILD_TESTING)' part to this 

```
    if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    find_package(ament_cmake_gtest REQUIRED)

    set(TESTFILES 
        test/main.cpp
        test/car_test.cpp
        )
    
    ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
    target_include_directories(node1 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

    install(TARGETS
        ${PROJECT_NAME}_test
        # Install the generated test executable file to the path after DESTINATION
        DESTINATION lib/${PROJECT_NAME})

    # ament_lint_auto_find_test_dependencies()
    endif()
```

An example project can be found in the assets folder or on this [link]().

![Screenshot file structure](../assets/environment_setup_tutorial/Screenshot-example-filestructure.png)