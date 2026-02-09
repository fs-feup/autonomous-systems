#include "fov_trimming/fov_trimming.hpp"

bool FovTrimming::within_limits(float x, float y, float z, float intensity,
                                const TrimmingParameters& params, const double max_range) const {
  bool ret = true;

  if (params.apply_fov_trimming) {
    const double angle = std::atan2(y, x) * 180.0 / M_PI;
    const double half_fov = params.fov / 2.0;
    if (angle < -half_fov || angle > half_fov) {
      ret &= false;
    }
  }

  double distance_squared = x * x + y * y;

  ret &= z < (params.max_height - params.lidar_height);
  ret &= (distance_squared > params.min_range * params.min_range) &&
         (distance_squared <= max_range * max_range);

  if (params.is_raining) {
    // In rainy conditions, the intensity must be at greater or equal to 1 to be considered valid,
    // as the water droplets cause 0 intensity points. It is not applied in normal conditions to
    // preserve more points, as a lot of cone points have low intensity.
    ret &= intensity >= params.minimum_intensity;
  }

  return ret;
}
