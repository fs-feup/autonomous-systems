#pragma once

#include <string>

struct VEParameters {
  std::string _estimation_method_;  // Used to choose between different velocity estimation methods
  std::string _adapter_;  // Used to choose between different adapters, e.g. pacsim to run the code
                          // on pacsim simulator
  double _ekf_process_noise_;  // Process noise for the EKF prediction step
};
