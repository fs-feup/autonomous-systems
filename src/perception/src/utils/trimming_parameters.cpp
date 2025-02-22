#include <complex>
#include <utils/trimming_parameters.hpp>

void TrimmingParameters::set_acceleration() {
  fov_trim_angle = 90;
  max_range = 20.25;
}
void TrimmingParameters::set_skidpad() {
  max_range = 20.25;
  // Calculate fov_trim_angle from the given minimum distance to a cone.
  fov_trim_angle = 90 - std::acos(1.5 / std::max(min_distance_to_cone, 1.5)) * 180 / M_PI;
}