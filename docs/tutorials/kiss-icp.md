# Kiss-icp

## Install kiss-icp pipeline


```sh
pip install kiss-icp
```

Next to test if the system is working are to understand instructions on how to run it do the following command

```sh
kiss_icp_pipeline --help
```

# Install kiss-icp ROS wrappers

From the src folder:

```sh
git submodule init
git submodule update --recursive --remote
```

Now go to the wd and build it by running the following command:

```sh
colcon build
```

If you are having problems with other packages, you can try to only build the kiss-icp

```sh
colcon build --packages-select kiss_icp
```

Finally run the following command to finish the build

```sh
source ./install/setup.bash
```

# How to run

The only required argument to provide is the topic name so KISS-ICP knows which PointCloud2 to process:

```sh
ros2 launch kiss_icp odometry.launch.py bagfile:=<path_to_rosbag> topic:=<topic_name>
```

You can optionally launch the node with any bagfile, and play the bagfiles on a different shell:
```sh
ros2 launch kiss_icp odometry.launch.py topic:=<topic_name>
```
and then,

```sh
ros2 bag play <path>*.bag
```

The topic_name should be the topic of the bagfile. In case you do not know, run the following commands and see which topic has the lidar data from the bagfile, and then repeat the commands above:

```sh
ros2 bag play <path>*.bag
ros2 topic list
```




To get the pose from kiss_icp, in a new terminal you should run:

```sh
ros2 topic echo /kiss/odometry
```

# Internal structure of the package

There are two nodes, one run kiss-icp and another used to just send data to rviz, in order to visualize the simulation.


The kiss-icp node is the node where the point cloud is recieved and the kiss-icp is executed. Inside this node there are several parameters that can be changed, from both ROS2 and kiss-icp.

**ROS2 parameters:** topic, bagfile, visualize, odom_frame, base_frame, publish_odom_tf.

**Kiss-icp parameters:** deskew, max_range, min_range


The rviz2 node is a simple node that only serves to visualize the map, and can read from a bagfile.

To have a better understanding of kiss-icp see this git repository https://github.com/PRBonn/kiss-icp?tab=readme-ov-file


