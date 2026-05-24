# InvictaSim Mesh Assets

Car, wheel, cone, and ground meshes for marker-based visualization in RViz/Foxglove.

## Car Meshes

Each car model has its own folder under `car/<model>/`. The folder name must match the
`car_parameters_config` selected by the simulator.

Required files for each car:

- `car_body.glb`
- `steering.glb`
- `wheel_front.glb`
- `wheel_back.glb`
- `pos.yaml`

The vehicle marker loader reads meshes from:

```text
package://invictasim/resources/meshes/car/<model>/<mesh_name>
```

## Position Configuration

`pos.yaml` stores visual alignment offsets for model-specific meshes:

```yaml
positions:
  steering_offset_x: 0.0
  steering_offset_y: 0.0
  steering_offset_z: 0.0
  steering_rotation_x: 0.0
  steering_rotation_y: 0.0
  steering_rotation_z: 0.0
  wheels_offset_x: 0.0
  wheels_offset_y: 0.0
  wheels_offset_z: 0.0
```

`steering_offset_*` moves the steering wheel mesh in the car frame.
`steering_rotation_*` is specified in degrees and is converted to radians by the loader.
`wheels_offset_*` is added to all four computed wheel positions, which is useful when the
wheel meshes are exported with a model-specific origin offset.

Missing values default to `0.0`.

## Other Meshes

Cone meshes are stored in `cones/`.
The ground plane mesh and texture are stored in `ground/`.
