# Perception Module

## Run the Node

### Compile

```SHELL
	colcon build --symlink-install --packages-select perception
```

### Run

```SHELL
	ros2 run perception perception
```

## Architecture

<p align="center">
  <img src="../../docs/assets/Perception/architecture.png" />
</p>


## Main External Libraries

1. [PyTorch](https://pytorch.org/tutorials/)
2. [ROS](https://docs.ros.org/en/foxy/index.html)
3. [CV2](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
4. [Numpy](https://numpy.org/doc/stable/)