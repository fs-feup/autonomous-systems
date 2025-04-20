#pragma once

struct LandmarkFilterParameters {
  int minimum_observation_count_ = 3;
  double minimum_frequency_of_detections_ = 5;

  LandmarkFilterParameters() = default;

  LandmarkFilterParameters(const int minimum_observation_count,
                           const double minimum_frequency_of_detections)
      : minimum_observation_count_(minimum_observation_count),
        minimum_frequency_of_detections_(minimum_frequency_of_detections) {}
};