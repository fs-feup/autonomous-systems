# InvictaSim Keys

## Package Information

### Description

This package provides a "Control Mock" for InvictaSim, allowing control of the simulator vehicle using the keyboard. It publishes commands to the simulator topics for testing models.

## How to Run

### Install Dependencies

```sh
./dependencies_install.sh
```

### Compiling
    
```sh
colcon build --packages-up-to invictasim_keys
```

### Running

To run the control mock, use:

```sh
ros2 run invictasim_keys invictasim_keys
```
