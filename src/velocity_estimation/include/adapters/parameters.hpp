#pragma once

#include <string>

struct VEParameters {
  std::string _estimation_method_;
  std::string _adapter_;
  double _ekf_process_noise_;
};
