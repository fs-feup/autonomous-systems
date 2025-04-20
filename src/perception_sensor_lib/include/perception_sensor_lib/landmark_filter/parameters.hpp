#pragma once

struct LandmarkFilterParameters {
  int minimum_number_of_observations = 3;
  int minimum_frequency = 5;

  LandmarkFilterParameters() = default;

  LandmarkFilterParameters(const double minimum_number_of_observations,
                           const double minimum_frequency)
      : minimum_number_of_observations(minimum_number_of_observations),
        minimum_frequency(minimum_frequency) {}
};