# Evaluator Package

This package contains the evaluator node, used to evaluate the AS main pipeline, with aid of simulation environments and other data.

## Compilation instructions

```sh
colcon build --symlink-install --packages-select eufs_msgs fs_msgs custom_interfaces pacsim
```

## Launch files

There are multiple launch files, each for each simulation environment.


## Running

Run a launch file:

```sh
ros2 launch evaluator evaluator_eufs.launch.py
```
You can run the node directly but it will use default settings.