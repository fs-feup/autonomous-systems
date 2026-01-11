#pragma once

#include <string>

struct FsfsimParameters {
  // Global config parameters
  std::string discipline;
  std::string track_name;
  double simulation_speed;

  // Simulator config parameters
  double timestep;
  std::string vehicle_model;

  FsfsimParameters() = default;
};
