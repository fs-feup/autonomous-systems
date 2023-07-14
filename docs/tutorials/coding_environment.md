# Coding environment

This tutorial aims to aid in teaching the developer how to use multiple tools established for this project, as well as creating the base environment for the project and understanding the project's structure.

### Note:
Before proceeding with this guide, please ensure that you have the following prerequisites in place:

- [ROS2 and Simulator Setup](sim_setup_tutorial.md#Nonfunctional-requirements#Colcon-setup)


## Links

- [SSH to Virtual Machine](https://averagelinuxuser.com/ssh-into-virtualbox/)
- [VSCode](https://code.visualstudio.com/Download)

## Cloning the project

Before anything else, you need to clone the project into your laptop using [git](https://git-scm.com/book/en/v2/Getting-Started-About-Version-Control). Make sure you setup an ssh connection with the repository. For more details on that, checkout the Github tutorials: [Generating a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) and [Adding a new SSH key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).

## Project Structure

This project is divided into multiple folders inside **src** folder. Each of these folders contain a ROS2 package. Essentially, the project is composed of multiple ROS2 packages whose nodes interact with each other in runtime. More details on the system's architecture can be found in [here](../project-specification.md).

### Notes on C++ Modules

- Each module is divided into three folders:
  - **src:** executables *.cpp files
  - **include:** libraries and headers *.hpp files
  - **test:** test files
- CMakeLists.txt is used as a build system (ROS2 default): it is a tool used to generate makefiles for c++. Everytime you creat e a new .cpp file, it must be added to the list of executable files in this CMakeLists.txt

## Coding Environment

In order to properly contribute to this project, a code editor or IDE is suggested. In this tutorial, some **suggestions** for an environment will be presented.


### VSCode with ssh connection

Visual Studio Code is a general purpose IDE very widely used. VSCode is our choice due to the great quantity of extensions available, namely:
- extensions for ROS environment and ROS2 syntax
- extensions for C++ and Python syntax check
- extensions for remote development via ssh (in Virtual Machines, for instance)

As the simulator required specific versions of Ubuntu to function, the setup chosen takes into account developping in the host machine while the project is held in the virtual machine, for faster typing response.

**Steps:**

- Install [VSCode](https://code.visualstudio.com/Download) on HostMachine
- [Install SSH server in virtual machine](https://averagelinuxuser.com/ssh-into-virtualbox/)
- Install vscode extensions in host machine:
    - [SSH Connection 1](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh)
    - [SSH Connection 2](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh-edit)
    - [SSH Connection 3](https://marketplace.visualstudio.com/items?itemName=ms-vscode.remote-explorer)
- (Optional) Install VSCode Extensions in Virtual Machine
    - CMake extension 
    - C++ Intellisense extension

### VSCode without ssh connection

As an alternative to the latter suggestion, one can install Visual Studio Code in the virtual machine and use it from there.

**Steps:**

- Install [VSCode](https://code.visualstudio.com/Download) on HostMachine
- (Optional) Install VSCode Extensions in Virtual Machine
    - CMake extension 
    - C++ Intellisense extension

## File Strucure and Guidelines

**THIS SECTION IS ONLY MEANT FOR THE CREATION OF NEW PARTS OF THE PROJECT**

A package in ROS2 can be created via the command line. The created project already constructs a certain file structure. However, some changes and additions ought to be made in order for it to meat the guidelines established. To create a project:
1. Run ```ros2 pkg create --build-type "ament_cmake" --node-name "<node_name>" "<package_name>"```
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

