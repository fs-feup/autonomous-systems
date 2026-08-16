#include "utils/path_stager.hpp"

#include <algorithm>

unsigned int get_s_index(std::vector<double>& s, double query_s) {
  // Simple linear search (can be optimized with binary search if needed)
  for (size_t i = 0; i < s.size() - 1; ++i) {
    if (s[i] > query_s) {
      return i - 1;
    }
  }
  return s.size() - 2; // Return the last valid index if out of bounds
}

double acceleration_for_segment(unsigned int start_idx, std::vector<double>& v, std::vector<double>& s) {
  double distance = s[start_idx + 1] - s[start_idx];
  double v_start = v[start_idx];
  double v_end = v[start_idx + 1];

  if (distance > 0.01) {
    return (v_end * v_end - v_start * v_start) / (2 * distance);
  } else {
    return 0.0; // Avoid division by zero, treat as no acceleration
  }
}

unsigned int compute_starting_index(const custom_interfaces::msg::PathPointArray& path_msg, const custom_interfaces::msg::VehicleStateVector& vehicle_state) {

  size_t closest_idx = -1;

  double x = vehicle_state.x;
  double y = vehicle_state.y;

  // Find the closest point to the vehicle
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path_msg.pathpoint_array.size(); ++i) {
    double dx = path_msg.pathpoint_array[i].x - x;
    double dy = path_msg.pathpoint_array[i].y - y;
    double distance = std::sqrt(dx * dx + dy * dy);
    if (distance < min_distance) {
      min_distance = distance;
      closest_idx = i;
    }
  }

  // Start from the previous point for better continuity
  if (closest_idx > 0 && closest_idx < path_msg.pathpoint_array.size() - 1) {
    double prev_dx = path_msg.pathpoint_array[closest_idx - 1].x - x;
    double prev_dy = path_msg.pathpoint_array[closest_idx - 1].y - y;
    double next_dx = path_msg.pathpoint_array[closest_idx + 1].x - x;
    double next_dy = path_msg.pathpoint_array[closest_idx + 1].y - y;
    double next_sq_distance = next_dx * next_dx + next_dy * next_dy;
    double prev_sq_distance = prev_dx * prev_dx + prev_dy * prev_dy;
    if (prev_sq_distance < next_sq_distance) closest_idx--;
  }

  return closest_idx;
}

void limit_velocity_according_to_current(custom_interfaces::msg::PathPointArray& path_msg, double max_acceleration = 10.0) {
  if (path_msg.pathpoint_array.empty()) {
    return;
  }

  // Limit velocity segment-by-segment using the previous point as the starting state
  // Start from the vehicle's current pose (front of the path) and current_velocity
  custom_interfaces::msg::PathPoint prev_point = path_msg.pathpoint_array.front();
  double prev_velocity = prev_point.v;

  for (size_t i = 0; i < path_msg.pathpoint_array.size(); ++i) {
    double dx = path_msg.pathpoint_array[i].x - prev_point.x;
    double dy = path_msg.pathpoint_array[i].y - prev_point.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    double max_velocity = std::sqrt(std::max(0.0, prev_velocity * prev_velocity + 2 * max_acceleration * distance));
    if (path_msg.pathpoint_array[i].v > max_velocity) {
      path_msg.pathpoint_array[i].v = max_velocity;
    }

    // Use the velocity and point as the previous for the next iteration
    prev_velocity = path_msg.pathpoint_array[i].v;
    prev_point = path_msg.pathpoint_array[i];
  }
}

void local_path_resampled_with_spline(custom_interfaces::msg::PathPointArray& path_msg, const custom_interfaces::msg::VehicleStateVector& vehicle_state, std::shared_ptr<LocalPather> local_pather, unsigned int number_of_stages, double horizon_length_seconds, custom_interfaces::msg::PathPointArray& output_path_data, bool free_speed) {
  if (path_msg.pathpoint_array.size() == 0) {
    return;
  }
  unsigned int starting_idx = compute_starting_index(path_msg, vehicle_state);

  size_t received_point_count = path_msg.pathpoint_array.size();
  size_t limit = received_point_count;

  if (path_msg.is_map_closed) {
    limit = starting_idx + received_point_count;
  }

  // Create new path starting from the starting index
  custom_interfaces::msg::PathPointArray new_path;
  for (size_t i = starting_idx; i < limit; ++i) {
    new_path.pathpoint_array.push_back(path_msg.pathpoint_array[i % received_point_count]);
  }

  local_pather->create_local_path(new_path, vehicle_state);

  double max_acceleration = 10.0; // m/s^2, can be tuned or made a parameter
  limit_velocity_according_to_current(new_path, max_acceleration);

  double car_x = new_path.pathpoint_array[0].x;
  double car_y = new_path.pathpoint_array[0].y;
  double car_yaw = new_path.pathpoint_array[0].orientation;
  double car_v = new_path.pathpoint_array[0].v;
  double car_yaw_rate = vehicle_state.yaw_rate;
  double max_breaking_acceleration = 6.0; // m/s^2
  double max_yaw_acceleration = 1.0; // rad/s^2
  double dt = horizon_length_seconds / static_cast<double>(number_of_stages);
  output_path_data.pathpoint_array.clear();

  // Default to just breaking until stopped
  if (new_path.pathpoint_array.size() < 3) {
    double time_to_stop = std::max(std::fabs(car_v / max_breaking_acceleration), std::fabs(car_yaw_rate / max_yaw_acceleration));

    double effective_acceleration = time_to_stop == 0 ? 0 : car_v / time_to_stop;
    double effective_yaw_acceleration = time_to_stop == 0 ? 0 : car_yaw_rate / time_to_stop;

    for (unsigned int i = 0; i <= number_of_stages; ++i) {
      custom_interfaces::msg::PathPoint p;
      p.x = car_x;
      p.y = car_y;
      p.orientation = car_yaw;
      p.v = car_v;
      output_path_data.pathpoint_array.push_back(p);

      // Update
      double next_v = car_v - effective_acceleration * dt;
      next_v = (car_v >= 0.0) ? std::max(0.0, next_v) : std::min(0.0, next_v);
      double next_yaw_rate = car_yaw_rate - effective_yaw_acceleration * dt;
      next_yaw_rate = (car_yaw_rate >= 0.0) ? std::max(0.0, next_yaw_rate) : std::min(0.0, next_yaw_rate);
      double average_v = (car_v + next_v) / 2.0;
      double average_yaw_rate = (car_yaw_rate + next_yaw_rate) / 2.0;

      car_x += std::cos(car_yaw) * average_v * dt;
      car_y += std::sin(car_yaw) * average_v * dt;
      car_yaw += average_yaw_rate * dt;
      car_v = next_v;
    }
    return;
  }

  // --- 1. Extract cumulative distance (s) from your existing local path ---
  std::vector<double> s, x, y, velocities;
  double current_s = 0.0;

  car_x = new_path.pathpoint_array[0].x;
  car_y = new_path.pathpoint_array[0].y;
  car_v = new_path.pathpoint_array[0].v;
  car_yaw = new_path.pathpoint_array[0].orientation;

  // Point 0: The actual car
  s.push_back(0.0);
  x.push_back(car_x);
  y.push_back(car_y);
  velocities.push_back(car_v);

  double dist_to_next = std::hypot(new_path.pathpoint_array[1].x - car_x, 
                                   new_path.pathpoint_array[1].y - car_y);
  double lookahead_dist = 0.1 * dist_to_next;

  // Interpolate the velocity assuming constant acceleration over the segment
  double next_velocity = new_path.pathpoint_array[1].v;
  double ghost_v = car_v;
  if (dist_to_next > 0.0) {
    double acceleration = (next_velocity * next_velocity - car_v * car_v) / (2.0 * dist_to_next);
    double ghost_v_sq = car_v * car_v + 2.0 * acceleration * lookahead_dist;
    ghost_v = std::sqrt(std::max(0.0, ghost_v_sq));
  }

  // Point 1: THE GHOST POINT
  current_s += lookahead_dist;
  s.push_back(current_s);
  x.push_back(car_x + std::cos(car_yaw) * lookahead_dist);
  y.push_back(car_y + std::sin(car_yaw) * lookahead_dist);
  velocities.push_back(ghost_v);

  // Point 2 to N: The rest of the macro path
  for (size_t i = 1; i < new_path.pathpoint_array.size(); ++i) {
    double dx = new_path.pathpoint_array[i].x - x.back();
    double dy = new_path.pathpoint_array[i].y - y.back();
    double dist = std::hypot(dx, dy);
   
    // Ensure the global points are far enough past the Ghost Point to avoid NaN crashes
    if (dist > 0.1) {
      current_s += dist;
      s.push_back(current_s);
      x.push_back(new_path.pathpoint_array[i].x);
      y.push_back(new_path.pathpoint_array[i].y);
      velocities.push_back(new_path.pathpoint_array[i].v);
    }
  }

  // Scale the reference profile by the car's speed relative to the plan, so a car running fast
  // does not outrun its own reference within the horizon.
  if (free_speed && velocities.size() > 2) {
    const double planner_v = velocities[2];
    if (planner_v > 0.5) {
      // Never scale below 1.0: shrinking the reference removes the signal to speed back up.
      const double scale = std::clamp(velocities[0] / planner_v, 1.0, 1.8);
      for (size_t i = 1; i < velocities.size(); ++i) {
        velocities[i] *= scale;
      }
    }
  }

  // --- 2. Fit the splines ---
  tk::spline spline_x, spline_y;

  // FORCE the spline's starting derivative to perfectly match the car's exact unit vector
  spline_x.set_boundary(tk::spline::first_deriv, std::cos(car_yaw), tk::spline::second_deriv, 0.0);
  spline_y.set_boundary(tk::spline::first_deriv, std::sin(car_yaw), tk::spline::second_deriv, 0.0);

  // Shape-preserving Hermite rather than C^2 cubic: the latter overshoots into a spurious
  // heading hook at corner entry when the car sits off the path, and the MPC follows it off.
  spline_x.set_points(s, x, tk::spline::cspline_hermite);
  spline_y.set_points(s, y, tk::spline::cspline_hermite);

  // --- 3. Sample the splines based on MPC dt ---    
  current_s = 0.0;
  double max_s = s.back();

  for (unsigned int i = 0; i <= number_of_stages; ++i) {
    custom_interfaces::msg::PathPoint p;

    // Clamp to prevent querying outside the spline bounds
    p.x = spline_x(current_s);
    p.y = spline_y(current_s);

    // Analytically compute perfectly smooth yaw from the geometry tangent
    double dx_ds = spline_x.deriv(1, current_s);
    double dy_ds = spline_y.deriv(1, current_s);
    p.orientation = std::atan2(dy_ds, dx_ds);
    p.v = car_v;
    output_path_data.pathpoint_array.push_back(p);

    // --- FINAL 2nd-Order Kinematic Integration ---
    unsigned int segment_start_idx = get_s_index(s, current_s);
    double acceleration = acceleration_for_segment(segment_start_idx, velocities, s);
    double end_of_segment_s = s[segment_start_idx + 1];

    double remaining_dt = dt;
    double remaining_dt2 = remaining_dt * remaining_dt;
    double ds = (car_v * remaining_dt) + (0.5 * acceleration * remaining_dt2);

    while (current_s + ds > end_of_segment_s) {
      double delta_s = end_of_segment_s - current_s;
      double average_speed = (car_v + velocities[segment_start_idx + 1]) / 2.0;
      double dt_until_segment_end = (average_speed > 1e-3) ? (delta_s / average_speed) : remaining_dt;
      
      current_s = end_of_segment_s;
      car_v = velocities[segment_start_idx + 1];
      segment_start_idx++;

      remaining_dt -= dt_until_segment_end;
      remaining_dt2 = remaining_dt * remaining_dt;
      end_of_segment_s = s[segment_start_idx + 1];
      acceleration = acceleration_for_segment(segment_start_idx, velocities, s);
      ds = (car_v * remaining_dt) + (0.5 * acceleration * remaining_dt2);
    }

    car_v += acceleration * remaining_dt;

    // 3. SAFETY CLAMPS
    // Prevent moving backward
    ds = std::max(ds, 0.0); 

    // Step forward, but STRICTLY LOCK to the end of the macro path
    current_s = std::min(current_s + ds, max_s); 
  }
}
