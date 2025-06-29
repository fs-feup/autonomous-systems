# Perception Sensor Library

## Package Information

### Description

This package serves as a library for definition of functions and structures that can treat LiDAR and perception data and may be both used in Perception, SLAM and other packages.

### Folder Structure

Package is called **perception_sensor_lib** and has no executable. 

It is composed of multiple namespaces:


### Important Dependencies

- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)


## How to Run

### Install Dependencies

```sh
  ./dependencies_install.sh
```

### Compiling

```sh
colcon build --packages-up-to perception_sensor_lib
```

### Testing

```sh
colcon test --packages-select perception_sensor_lib # use event-handler=console_direct+ for imediate output
```

To check test results:
```sh
colcon test-result --all --verbose
```

