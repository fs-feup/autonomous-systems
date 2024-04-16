
# general
sudo apt-get update && apt-get upgrade -y
sudo apt-get install -y python3-pip
sudo apt install python-is-python3 -y
sudo apt install ros-humble-ackermann-msgs -y
sudo apt install -y python3-pip
sudo pip3 install colcon-common-extensions
sudo apt install -y python3-rosdep

# planning
sudo apt-get install libcgal-dev -y
sudo apt-get install libgsl-dev -y


# perception
sudo apt install libpcl-dev -y
sudo apt-get install -y ros-sensor-msgs -y
sudo apt install libsensor-msgs-dev -y
sudo apt install libpcl-conversions-dev -y
sudo apt-get install ros-humble-pcl-conversions -y
sudo apt-get install ros-humble-pcl-ros -y
sudo apt install rospack-tools -y
rospack find sensor_msgs -y
sudo apt-get install libpcap-dev -y

source /opt/ros/humble/setup.bash