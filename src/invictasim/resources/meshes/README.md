# InvictaSim Mesh Assets

Car, wheel, cone, and ground meshes for marker-based visualization in Foxglove.

## Car Meshes

Each car model has its own folder under `car/<model>/`. The folder name must match the
`car_parameters_config` selected by the simulator.

Required files for each car:

- `car_body.glb`
- `steering.glb`
- `wheel_front.glb`
- `wheel_back.glb`
- `config.yaml`

The vehicle marker loader reads meshes from:

```text
package://invictasim/resources/meshes/car/<model>/<mesh_name>
```

## Position Configuration

`config.yaml` stores visual alignment offsets for model-specific meshes:

```yaml
positions:
  steering_offset_x: 0.0
  steering_offset_y: 0.0
  steering_offset_z: 0.0
  steering_rotation_x: 0.0
  steering_rotation_y: 0.0
  steering_rotation_z: 0.0
  steering_rotation_multiplier: 1.0
  wheels_offset_x: 0.0
  wheels_offset_y: 0.0
  wheels_offset_z: 0.0
```

`steering_offset_*` moves the steering wheel mesh in the car frame.
`steering_rotation_*` is specified in degrees and is converted to radians by the loader.
`wheels_offset_*` is added to all four computed wheel positions, which is useful when the
wheel meshes are exported with a model-specific origin offset.

Missing values default to `0.0`.

## Hitbox Configuration

`config.yaml` can also define the 2D car hitboxes used for cone-hit detection:

```yaml
hitboxes:
  visualize: false
  boxes:
    - name: chassis
      center_x: 0.0
      center_y: 0.0
      length: 1.5
      width: 1.4
    - name: front_wing
      center_x: 1.1
      center_y: 0.0
      length: 0.3
      width: 1.6
```

`hitboxes.visualize` controls whether these rectangles are drawn in the car marker array. It does not disable collision checks.

`hitboxes.boxes` contains the rectangles used by collision detection.

Hitboxes are axis-aligned rectangles in the car frame, in meters. `x` points forward, `y` points left, and cone collision is checked against the union of all configured rectangles. If no hitbox boxes are configured, no car-cone collision is detected.

## Cone Configuration

`cones/config.yaml` stores cone collision radii and optional radius visualization:

```yaml
visualization:
  hitboxes:
    visualize: false
    z: 0.02
    height: 0.04
    alpha: 0.35

collision:
  standard_radius: 0.115
  large_radius: 0.15
  hit_match_distance: 0.35
```

`standard_radius` and `large_radius` are used by car-cone collision detection. `hit_match_distance` is used only to recolor recently hit cones in the visualization.

## Ground Configuration

`ground/config.yaml` stores the ground marker, start-line marker, and lap-finish thresholds:

```yaml
visualization:
  position: {x: 0.0, y: 0.0, z: -0.02}
  scale: {x: 1000.0, y: 1000.0, z: 1.0}
  color: {r: 0.78, g: 0.78, b: 0.78, a: 1.0}
  start_line:
    target_cell_length: 0.5
    row_count: 2
    total_width: 0.45
    z: 0.01
    height: 0.02

lap_finish:
  start_line_gate_margin: 2.0
  minimum_lap_time: 3.0
  minimum_lap_distance: 20.0
  penalties:
    default_cone: 2.0
    skidpad_cone: 0.2
```

## Other Meshes

Cone meshes are stored in `cones/`.
The ground plane mesh and texture are stored in `ground/`.
