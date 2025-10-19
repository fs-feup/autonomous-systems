#! /bin/bash
# general
sudo apt-get update && apt-get upgrade -y
sudo apt-get install -y python3-pip
sudo apt install python-is-python3 -y
sudo apt install ros-humble-ackermann-msgs -y
sudo apt install -y python3-pip
sudo pip3 install colcon-common-extensions
sudo apt install -y python3-rosdep
sudo apt install -y ros-humble-message-filters
sudo apt install -y ros-humble-tf-transformations
sudo apt install -y ros-humble-rosbag2-storage-mcap
sudo apt install ninja-build build-essential cmake -y

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

# ground truth generation
pip install pandas
sudo apt-get install python3-matplotlib -y

# evaluator
pip install transforms3d

#cloud storage
pip install google-cloud-storage

#local dashboard
pip install dash

# needed to not broke evaluator
pip install numpy==1.21.5

pip install transforms3d==0.4.1
pip install scipy==1.10.0