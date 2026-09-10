#pragma once

#include "utils/spline.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "local_pather/map.hpp"

/**
 * @brief Resample the global path into one reference point per MPC stage.
 *
 * @param free_speed Scale the reference profile by the car's actual speed, for controllers
 *   that choose their own pace and would otherwise outrun the reference.
 */
void local_path_resampled_with_spline(custom_interfaces::msg::PathPointArray& path_msg, const custom_interfaces::msg::VehicleStateVector& vehicle_state, std::shared_ptr<LocalPather> local_pather, unsigned int number_of_stages, double horizon_length, custom_interfaces::msg::PathPointArray& output_path_data, bool free_speed = false);