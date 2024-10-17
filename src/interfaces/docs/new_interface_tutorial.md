# Custom ROS 2 Interfaces Tutorial

This tutorial is meant to aid the developer create new custom interfaces in the ROS2 program for specific topics.

## Links

- [ROS2 custom interfaces tutorial documentation (Crystal distro)](https://docs.ros.org/en/crystal/Tutorials/Custom-ROS2-Interfaces.html#create-a-new-package)
  - [Other good tutorial with video](https://roboticsbackend.com/ros2-create-custom-message/)
- [Set of packages with common ROS2 interfaces](https://github.com/ros2/common_interfaces)
- [ROS2 Field Types for interfaces (Galactic Distro)](https://docs.ros.org/en/galactic/Concepts/About-ROS-Interfaces.html#field-types)

## Keywords

- ROS - Robot Operating System
- ROS2 - New version of ROS that will be used
- Interface / Msg - The content which is sent throught ROS2 topics or any other type of communication channel

## 1. Create package

It is essential you create an independent package for the purpose of the tutorial. (Note: the project may already include one, you are not supposed to create multiple packages for new interfaces)

```sh
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```

`tutorial_interfaces` is the name of the newly introduced package. Note that it is a CMake package since there is no way to generate a `.msg` or `.srv` file in a pure Python package.

It is good practice to keep `.msg` and `.srv` files in their own directories within a package. So we must create the respective folders:

```sh
mkdir msg

mkdir srv
```

## 2. Create custom definitions

In the `tutorial_interfaces/msg` directory you just created, make a new file called `Num.msg` with one line of code declaring its data structure:

```
int64 num
```

This will be our custom message, and it transfers a single 64-bit integer called `num`.

There is a set of other primitive types with the respective Python & C++ conversions in the **Links section**.

### 2.1 Custom dependent definitions

If by any chance you wish to implement an interface which depends on other created interfaces, you may do it like so:

```
geometry_msgs/Point point
```

Here we are transferring a `Point` derived from the `geometry_msgs` package.

If we take a look at its definition, we conclude it is the same as writing the following piece of code:

```sh
# This contains the position of a point in free space
float64 x
float64 y
float64 z
```

### 2.2 Proper messages

To create proper messages, you must always include the **Header** type message, so that timestamps and other important details can be tracked.

```sh
# This contains the position of a point in free space
std_msgs/Header header
float64 x
float64 y
float64 z
```

## 3. Package.xml

Because the interfaces rely on `rosidl_default_generators` for generating language-specific code, you need to declare a dependency on it. Add the following lines to `package.xml`:

```xml
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

### 3.1 Custom dependent definitions

As you may know from other ROS2 experiments, if we use any external interface, we need to create a specific dependency for it. In the case we presented before, we would need to include the following line:

```xml
<depend>geometry_msgs</depend>
```

## 4. CMakeLists.txt

To convert the interfaces you defined into language-specific code (like C++ and Python) so that they can be used in those languages, add the following lines to `CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)

# Add any new custom interface here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
)

ament_export_dependencies(rosidl_default_runtime)
```

### 4.1 Custom dependent definitions

In addition to the above lines, in case we are using dependent definitions, we must add the correspondent `find_package` statement and add it to the `rosidl_generate_interfaces` like so:

```cmake
find_package(geometry_msgs REQUIRED)

# Add any new custom interface here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  DEPENDENCIES geometry_msgs
)
```

## 5. Build the package

Now that all the parts of your custom interfaces package are in place, you can build the package:

```sh
colcon build --packages-select tutorial_interfaces
```

## 6. Confirm msg and srv creation

In a new terminal, run the following command from within your workspace to source it:

```sh
. install/setup.bash
```

Now you can confirm that your interface creation worked by using the command:

```sh
ros2 interface show tutorial_interfaces/msg/Num
```

With the desired display:

```
int64 num
```

## 7. Test the new interfaces

The new interfaces can be used now in a real Python/C++ package. You may do so by importing the respective `.msg` file:

**Python:**
```py
from tutorial_interfaces.msg import Num
```
**C++:**
```cpp
#include "tutorial_interfaces/msg/num.hpp"
```

And not forget to complete the `package.xml` / `CMakeLists.txt` files.

## 8. SRV files

The `.srv` files may be used to create custom service messages. However, the focus of this tutorial is on topic-related messaging, so we will not cover any further steps.

You may follow the first link which also guides this type of communication.
