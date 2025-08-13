section{System Version 0.3}

\subsection{Topics validation}

The topics related to the wss (wheel speed sensor) are not correctly validated. The graphs do not show values ​​about them.
Some other topics also don't shown any values like:

```bash
powerground_setpoint.value
perception/lidar/visualization.markers[:]{id==0}.colors[0].r
ts/voltage
torques_max.rl
map.markers[:]{id==0}.colors[0].r
```

The other topics are working as desired like:

```bash
/steering_setpoint.value
velocity.twist.twist.angular.x /y/z
gnss/gnss_front.altitude
steeringRear.value
steeringFront.value
torques.rl
imu/cog_imu.angular_velocity.x /y/z
imu/cog_imu.linear_acceleration.x /y/z
perception/lidar/landmarks.detections[0].pose.pose.position.x
gnss/gnss_front.velocity_north
gnss/gnss_front.longitude
mu/cog_imu.orientation.w
```

\subsubsection{Variables}

Correct initialization of variables: All relevant variables were properly initialized, ensuring the expected behavior of the system from the beginning of the simulation.
Consistency between variables and parameters: The consistency between the different variables of the model was verified, ensuring that there are no conflicts or logical inconsistencies between their values ​​and interdependencies.

wheelspeeds:

```bash
    rate: 200; 
    dead_time: 0.005
```

Vehicle_model:

```bash
    kinematics:
        lr: 0.804 
        lf: 0.726 
        sf: 1.2 
        sr: 1.2 
    m: 200.707
    wheelRadius: 0.203
    gearRatio: 4.0
    innerSteeringRatio: 0.1869
    outerSteeringRatio: 0.1582
```

Noise:

```bash
    position: 
      mean: [0.0, 0.0, 0.0]
      sigma: [0.03, 0.03, 1.3]
    velocity: 
      mean: [0.0, 0.0, 0.0]
      sigma: [0.02, 0.02, 0.02]
    orientation: 
      mean: [0.0, 0.0, 0.0]
      sigma: [0.02, 0.02, 0.02]
```

\subsubsection{Sensors}

Sensor noise compatible with the specified sensor: The noise attributed to the sensors in the model is compatible with the characteristics of the real sensor, ensuring that the simulated behavior adequately represents the uncertainties associated with the measurements.
