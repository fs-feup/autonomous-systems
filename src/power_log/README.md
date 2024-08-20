# Power Log

## Package Information

### Description

This package is for a python node used in vehicle to record diagnostics of AS CU's health, such as:
- RAM usage
- CPU core usage
- CPU temperature

### Important Dependencies

- [psutil](https://psutil.readthedocs.io/en/latest/)

## How to Run

### Install Dependencies

```sh
./dependencies_install.sh
```

### Compiling

```sh
colcon build --packages-up-to power_log
```

### Running

To run the power log, you need to run the following command:

```sh
ros2 run power_log power_log
```
