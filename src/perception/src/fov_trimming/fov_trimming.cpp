#include "fov_trimming/fov_trimming.hpp"

bool FovTrimming::within_limits(float x, float y, float z, const TrimmingParameters& params,
                                const double max_range) const {
  double distance_squared = x * x + y * y;

  const bool within_height = z < (params.max_height - params.lidar_height);
  const bool within_range = (distance_squared > params.min_range * params.min_range) &&
                            (distance_squared <= max_range * max_range);

  return within_height && within_range;
}
