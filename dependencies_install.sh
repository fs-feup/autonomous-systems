sudo apt update
sudo apt install ros-humble-ackermann-msgs -y

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


source /opt/ros/humble/setup.bash

