# Project Startup Guide

This document serves as a startup guide for individuals interested in contributing to this project. The guide aims to provide you with the necessary information to become familiar and comfortable with the project.

### Note:
Before proceeding with this guide, please ensure that you have the following prerequisites in place:

- [EUFS SIM Simulator Tutorial](sim_setup_tutorial.md#Nonfunctional-requirements#Colcon-setup)

## Steps to Follow

1. **Clone** the project using [git](https://git-scm.com/book/en/v2/Getting-Started-About-Version-Control)

2. **Navigate** to the project's "driverless" folder and access the "src" directory.
```
cd driverless/src
```

3. Once you have set up the environment, it's time to **compile** the project using [Colcon](sim_setup_tutorial.md#Nonfunctional-requirements#Colcon-setup)

If you only wish to **compile specific packages**, you can use the following command:
```
colcon build --packages-select [your package and others necessary]
```
Alternatively, to **compile everything**, execute:
```
colcon build
```

As a starting point, let's try to **compile** a single package, _control_master_. Run the following command:

```
colcon build --packages-select control_mpc
```

If the compilation is successful, you will see a **success message**.

4. Whenever you compile the project, you **must** run the following command:

```
source install/setup.bash
```

5. Now that the project is compiled, you are ready to **run** the project's nodes. We will once again use the *control_mpc* package.

To **run any node**, use the following command:

```
ros2 run [package_name] [node_name]
```

In our case, let's run the previously compiled package:

```
ros2 run control_mpc control_mpc
```

6. Now, attempt to compile all the packages and run all the nodes. Refer to the table below to help you identify the packages and nodes:

<br>

| Package     | Node        |
| ----------- | ----------- |
| control     | control_mpc | 
| loc_map     | loc_map     |

(finish the table)
