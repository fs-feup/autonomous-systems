# Variables
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

# ROS 2 paths (replace 'galactic' with your ROS 2 distro if different)
ROS2_INSTALL_PATH = /opt/ros/humble

# Include directories
INCLUDE_DIRS = \
    -I$(ROS2_INSTALL_PATH)/include/ \
    -I$(ROS2_INSTALL_PATH)/include/rclcpp/ \
    -I$(ROS2_INSTALL_PATH)/include/rcl/ \
    -I$(ROS2_INSTALL_PATH)/include/rcutils/ \
    -I$(ROS2_INSTALL_PATH)/include/rmw/ \
    -I$(ROS2_INSTALL_PATH)/include/rcl_yaml_param_parser/ \
    -I$(ROS2_INSTALL_PATH)/include/rosidl_runtime_c/ \
    -I$(ROS2_INSTALL_PATH)/include/rosidl_typesupport_interface/ \
    -I$(ROS2_INSTALL_PATH)/include/rcpputils/ \
    -I$(ROS2_INSTALL_PATH)/include/builtin_interfaces/ \
    -I$(ROS2_INSTALL_PATH)/include/rosidl_runtime_cpp/ \
    -I$(ROS2_INSTALL_PATH)/include/tracetools/ \
    -I$(ROS2_INSTALL_PATH)/include/rcl_interfaces/ \
    -I$(ROS2_INSTALL_PATH)/include/libstatistics_collector/ \
    -I$(ROS2_INSTALL_PATH)/include/statistics_msgs/ \
    -I/home/ws/src/planning/include \
    -I/home/ws/src/common_lib/include \
    -I/home/ws/src/planning/include/config

# Link directories
LIB_DIRS = \
    -L$(ROS2_INSTALL_PATH)/lib

# ROS 2 libraries (may need to expand this list)
LIBS = -lc -lstdc++ \
       -lrclcpp \
       -lrclcpp_lifecycle \
       -lrcutils \
       -lrmw \
       -lrosidl_typesupport_cpp


# Source files
SRC = /home/ws/bayesian-opt/cone_coloring/cpp-python-bridge.cpp \
      /home/ws/src/planning/src/planning/cone_coloring.cpp \
      /home/ws/src/common_lib/src/competition_logic/color.cpp \
      /home/ws/src/common_lib/src/maths/angle_and_norms.cpp \
      /home/ws/src/common_lib/src/structures/cone.cpp \
      /home/ws/src/common_lib/src/structures/pose.cpp \
      /home/ws/src/common_lib/src/structures/position.cpp

# Output binary
TARGET = bayesian_opt

# Build rules

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -o $(TARGET) $(SRC) $(LIB_DIRS) $(LIBS)

clean:
	rm -f $(TARGET)