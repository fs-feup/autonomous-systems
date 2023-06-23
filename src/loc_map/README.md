# Localization and Mapping Package

## Compiling

From src folder:
```sh
colcon build --packages-select loc_map custom_interfaces
```

## Testing

From src folder:
```sh
colcon test --packages-select loc_map
```

## Running the nodes
```sh
ros2 run loc_map loc_map
```

This command will launch 3 nodes:
- **loc_map_publisher:** publishes data
- **loc_map_subscriber:** subscribes the required topics
- **ekf_node:** runs the main SLAM algorithm
