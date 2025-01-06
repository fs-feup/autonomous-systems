#pragma once

#include <string>

struct VEParameters {
  std::string _estimation_method_;  // Used to choose between different velocity estimation methods
  std::string _adapter_;  // Used to choose between different adapters, e.g. pacsim to run the code
                          // on pacsim simulator
  double _ekf_process_noise_;          // Process noise for the EKF prediction step
  double _ekf_measurement_noise_;      // Measurement noise for the EKF correction step
  double _wheel_base_;                 // Distance between the front and rear axles
  double _weight_distribution_front_;  // Weight distribution on the front axle [0,1]
  double _gear_ratio_;                 // Gear ratio of the vehicle
  double _wheel_radius_;               // Radius of the wheel
};
