#pragma once

#include "utils/spline.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "local_pather/map.hpp"

/**
 * @brief Resample the global path into one reference point per MPC stage.
 *
 * @param free_speed When false the reference advances at the planner's velocity
 *   profile, snapping to it at every path segment. When true the reference is
 *   propagated from the car's ACTUAL speed using the planner's accelerations and
 *   never snaps back. Controllers that choose their own speed need the latter:
 *   otherwise a car running faster than planned outruns its own reference and
 *   the last stages point at ground it has already covered.
 */
void local_path_resampled_with_spline(custom_interfaces::msg::PathPointArray& path_msg, const custom_interfaces::msg::VehicleStateVector& vehicle_state, std::shared_ptr<LocalPather> local_pather, unsigned int number_of_stages, double horizon_length, custom_interfaces::msg::PathPointArray& output_path_data, bool free_speed = false);