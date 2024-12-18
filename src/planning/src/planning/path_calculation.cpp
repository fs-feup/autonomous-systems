#include "planning/path_calculation.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "utils/cone.hpp"

std::vector<PathPoint> PathCalculation::process_delaunay_triangulations(
    std::pair<std::vector<Cone>, std::vector<Cone>> refined_cones) const {
  // merge left and right cones for next step
  // RCLCPP_WARN(rclcpp::get_logger("planning"), "Refined cones: %d blue, %d yellow",
  //            static_cast<int>(refined_cones.first.size()),
  //            static_cast<int>(refined_cones.second.size()));
  std::vector<Cone> cones;
  cones.reserve(refined_cones.first.size() + refined_cones.second.size());
  cones.insert(cones.end(), refined_cones.first.begin(), refined_cones.first.end());
  cones.insert(cones.end(), refined_cones.second.begin(), refined_cones.second.end());

  std::vector<PathPoint> unordered_path;
  unordered_path.reserve(cones.size());

  // Create a Delaunay triangulation
  DT dt;
  // Insert cones coordinates into the Delaunay triangulation
  for (const Cone &cone : cones) {
    dt.insert(Point(cone.position.x, cone.position.y));
  }

  // Process valid triangulations and add positions to unordered_path
  for (DT::Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
    // Extract vertices' coordinates from both edges
    double x1 = it->first->vertex((it->second + 1) % 3)->point().x();
    double y1 = it->first->vertex((it->second + 1) % 3)->point().y();
    double x2 = it->first->vertex((it->second + 2) % 3)->point().x();
    double y2 = it->first->vertex((it->second + 2) % 3)->point().y();

    // Find corresponding cones for the vertices
    int id_cone1 = find_cone(cones, x1, y1);
    int id_cone2 = find_cone(cones, x2, y2);
    // Check both cones have been found
    if (id_cone1 == -1 || id_cone2 == -1) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cone not found in triangulations");
      continue;
    }

    Cone cone1 = cones[id_cone1];
    Cone cone2 = cones[id_cone2];
    // If cones are from different sides
    if (cone1.color != cone2.color) {
      // Calculate the midpoint between the two cones
      double x_dist = cone2.position.x - cone1.position.x;
      double y_dist = cone2.position.y - cone1.position.y;
      double dist = sqrt(pow(x_dist, 2) + pow(y_dist, 2));

      if (dist < config_.dist_threshold_) {
        PathPoint pt = PathPoint(cone1.position.x + x_dist / 2, cone1.position.y + y_dist / 2);
        unordered_path.push_back(pt);
      }
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Unordered path size: %d",
               static_cast<int>(unordered_path.size()));

  return unordered_path;
}

std::vector<PathPoint> PathCalculation::skidpad_path(std::vector<Cone> &cone_array,
                                                     common_lib::structures::Pose pose) {
  if (!path_orientation_corrected_) {
    // define the original predefined path
    double entryspeed = 6;
    double exitspeed = 8;
    double rotatingspeed = 5;
    std::vector<PathPoint> hardcoded_path_ = {PathPoint(0.000, 0, entryspeed),
                                              PathPoint(0.500, 0, entryspeed),
                                              PathPoint(1.000, 0, entryspeed),
                                              PathPoint(1.500, 0, entryspeed),
                                              PathPoint(2.000, 0, entryspeed),
                                              PathPoint(2.500, 0, entryspeed),
                                              PathPoint(3.000, 0, entryspeed),
                                              PathPoint(3.500, 0, entryspeed),
                                              PathPoint(4.000, 0, entryspeed),
                                              PathPoint(4.500, 0, entryspeed),
                                              PathPoint(5.000, 0, entryspeed),
                                              PathPoint(5.500, 0, entryspeed),
                                              PathPoint(6.000, 0, entryspeed),
                                              PathPoint(6.500, 0, entryspeed),
                                              PathPoint(7.000, 0, entryspeed),
                                              PathPoint(7.500, 0, entryspeed),
                                              PathPoint(8.000, 0, entryspeed),
                                              PathPoint(8.500, 0, entryspeed),
                                              PathPoint(9.000, 0, entryspeed),
                                              PathPoint(9.500, 0, entryspeed),
                                              PathPoint(10.000, 0, entryspeed),
                                              PathPoint(10.500, 0, entryspeed),
                                              PathPoint(11.000, 0, entryspeed),
                                              PathPoint(11.500, 0, entryspeed),
                                              PathPoint(12.000, 0, entryspeed),
                                              PathPoint(12.500, 0, entryspeed),
                                              PathPoint(13.000, 0, entryspeed),
                                              PathPoint(13.500, 0, entryspeed),
                                              PathPoint(14.000, 0, entryspeed),
                                              PathPoint(14.500, 0, entryspeed),
                                              PathPoint(15.000, -0.000, rotatingspeed),
                                              PathPoint(15.573, -0.018, rotatingspeed),
                                              PathPoint(16.144, -0.072, rotatingspeed),
                                              PathPoint(16.710, -0.162, rotatingspeed),
                                              PathPoint(17.269, -0.287, rotatingspeed),
                                              PathPoint(17.820, -0.447, rotatingspeed),
                                              PathPoint(18.359, -0.641, rotatingspeed),
                                              PathPoint(18.885, -0.868, rotatingspeed),
                                              PathPoint(19.396, -1.129, rotatingspeed),
                                              PathPoint(19.889, -1.421, rotatingspeed),
                                              PathPoint(20.364, -1.743, rotatingspeed),
                                              PathPoint(20.816, -2.094, rotatingspeed),
                                              PathPoint(21.246, -2.473, rotatingspeed),
                                              PathPoint(21.652, -2.879, rotatingspeed),
                                              PathPoint(22.031, -3.309, rotatingspeed),
                                              PathPoint(22.382, -3.761, rotatingspeed),
                                              PathPoint(22.704, -4.236, rotatingspeed),
                                              PathPoint(22.996, -4.729, rotatingspeed),
                                              PathPoint(23.257, -5.240, rotatingspeed),
                                              PathPoint(23.484, -5.766, rotatingspeed),
                                              PathPoint(23.678, -6.305, rotatingspeed),
                                              PathPoint(23.838, -6.856, rotatingspeed),
                                              PathPoint(23.963, -7.415, rotatingspeed),
                                              PathPoint(24.053, -7.981, rotatingspeed),
                                              PathPoint(24.107, -8.552, rotatingspeed),
                                              PathPoint(24.125, -9.125, rotatingspeed),
                                              PathPoint(24.107, -9.698, rotatingspeed),
                                              PathPoint(24.053, -10.269, rotatingspeed),
                                              PathPoint(23.963, -10.835, rotatingspeed),
                                              PathPoint(23.838, -11.394, rotatingspeed),
                                              PathPoint(23.678, -11.945, rotatingspeed),
                                              PathPoint(23.484, -12.484, rotatingspeed),
                                              PathPoint(23.257, -13.010, rotatingspeed),
                                              PathPoint(22.996, -13.521, rotatingspeed),
                                              PathPoint(22.704, -14.014, rotatingspeed),
                                              PathPoint(22.382, -14.489, rotatingspeed),
                                              PathPoint(22.031, -14.941, rotatingspeed),
                                              PathPoint(21.652, -15.371, rotatingspeed),
                                              PathPoint(21.246, -15.777, rotatingspeed),
                                              PathPoint(20.816, -16.156, rotatingspeed),
                                              PathPoint(20.364, -16.507, rotatingspeed),
                                              PathPoint(19.889, -16.829, rotatingspeed),
                                              PathPoint(19.396, -17.121, rotatingspeed),
                                              PathPoint(18.885, -17.382, rotatingspeed),
                                              PathPoint(18.359, -17.609, rotatingspeed),
                                              PathPoint(17.820, -17.803, rotatingspeed),
                                              PathPoint(17.269, -17.963, rotatingspeed),
                                              PathPoint(16.710, -18.088, rotatingspeed),
                                              PathPoint(16.144, -18.178, rotatingspeed),
                                              PathPoint(15.573, -18.232, rotatingspeed),
                                              PathPoint(15.000, -18.250, rotatingspeed),
                                              PathPoint(14.427, -18.232, rotatingspeed),
                                              PathPoint(13.856, -18.178, rotatingspeed),
                                              PathPoint(13.290, -18.088, rotatingspeed),
                                              PathPoint(12.731, -17.963, rotatingspeed),
                                              PathPoint(12.180, -17.803, rotatingspeed),
                                              PathPoint(11.641, -17.609, rotatingspeed),
                                              PathPoint(11.115, -17.382, rotatingspeed),
                                              PathPoint(10.604, -17.121, rotatingspeed),
                                              PathPoint(10.111, -16.829, rotatingspeed),
                                              PathPoint(9.636, -16.507, rotatingspeed),
                                              PathPoint(9.184, -16.156, rotatingspeed),
                                              PathPoint(8.754, -15.777, rotatingspeed),
                                              PathPoint(8.348, -15.371, rotatingspeed),
                                              PathPoint(7.969, -14.941, rotatingspeed),
                                              PathPoint(7.618, -14.489, rotatingspeed),
                                              PathPoint(7.296, -14.014, rotatingspeed),
                                              PathPoint(7.004, -13.521, rotatingspeed),
                                              PathPoint(6.743, -13.010, rotatingspeed),
                                              PathPoint(6.516, -12.484, rotatingspeed),
                                              PathPoint(6.322, -11.945, rotatingspeed),
                                              PathPoint(6.162, -11.394, rotatingspeed),
                                              PathPoint(6.037, -10.835, rotatingspeed),
                                              PathPoint(5.947, -10.269, rotatingspeed),
                                              PathPoint(5.893, -9.698, rotatingspeed),
                                              PathPoint(5.875, -9.125, rotatingspeed),
                                              PathPoint(5.893, -8.552, rotatingspeed),
                                              PathPoint(5.947, -7.981, rotatingspeed),
                                              PathPoint(6.037, -7.415, rotatingspeed),
                                              PathPoint(6.162, -6.856, rotatingspeed),
                                              PathPoint(6.322, -6.305, rotatingspeed),
                                              PathPoint(6.516, -5.766, rotatingspeed),
                                              PathPoint(6.743, -5.240, rotatingspeed),
                                              PathPoint(7.004, -4.729, rotatingspeed),
                                              PathPoint(7.296, -4.236, rotatingspeed),
                                              PathPoint(7.618, -3.761, rotatingspeed),
                                              PathPoint(7.969, -3.309, rotatingspeed),
                                              PathPoint(8.348, -2.879, rotatingspeed),
                                              PathPoint(8.754, -2.473, rotatingspeed),
                                              PathPoint(9.184, -2.094, rotatingspeed),
                                              PathPoint(9.636, -1.743, rotatingspeed),
                                              PathPoint(10.111, -1.421, rotatingspeed),
                                              PathPoint(10.604, -1.129, rotatingspeed),
                                              PathPoint(11.115, -0.868, rotatingspeed),
                                              PathPoint(11.641, -0.641, rotatingspeed),
                                              PathPoint(12.180, -0.447, rotatingspeed),
                                              PathPoint(12.731, -0.287, rotatingspeed),
                                              PathPoint(13.290, -0.162, rotatingspeed),
                                              PathPoint(13.856, -0.072, rotatingspeed),
                                              PathPoint(14.427, -0.018, rotatingspeed),
                                              PathPoint(15.000, -0.000, rotatingspeed),
                                              PathPoint(15.573, -0.018, rotatingspeed),
                                              PathPoint(16.144, -0.072, rotatingspeed),
                                              PathPoint(16.710, -0.162, rotatingspeed),
                                              PathPoint(17.269, -0.287, rotatingspeed),
                                              PathPoint(17.820, -0.447, rotatingspeed),
                                              PathPoint(18.359, -0.641, rotatingspeed),
                                              PathPoint(18.885, -0.868, rotatingspeed),
                                              PathPoint(19.396, -1.129, rotatingspeed),
                                              PathPoint(19.889, -1.421, rotatingspeed),
                                              PathPoint(20.364, -1.743, rotatingspeed),
                                              PathPoint(20.816, -2.094, rotatingspeed),
                                              PathPoint(21.246, -2.473, rotatingspeed),
                                              PathPoint(21.652, -2.879, rotatingspeed),
                                              PathPoint(22.031, -3.309, rotatingspeed),
                                              PathPoint(22.382, -3.761, rotatingspeed),
                                              PathPoint(22.704, -4.236, rotatingspeed),
                                              PathPoint(22.996, -4.729, rotatingspeed),
                                              PathPoint(23.257, -5.240, rotatingspeed),
                                              PathPoint(23.484, -5.766, rotatingspeed),
                                              PathPoint(23.678, -6.305, rotatingspeed),
                                              PathPoint(23.838, -6.856, rotatingspeed),
                                              PathPoint(23.963, -7.415, rotatingspeed),
                                              PathPoint(24.053, -7.981, rotatingspeed),
                                              PathPoint(24.107, -8.552, rotatingspeed),
                                              PathPoint(24.125, -9.125, rotatingspeed),
                                              PathPoint(24.107, -9.698, rotatingspeed),
                                              PathPoint(24.053, -10.269, rotatingspeed),
                                              PathPoint(23.963, -10.835, rotatingspeed),
                                              PathPoint(23.838, -11.394, rotatingspeed),
                                              PathPoint(23.678, -11.945, rotatingspeed),
                                              PathPoint(23.484, -12.484, rotatingspeed),
                                              PathPoint(23.257, -13.010, rotatingspeed),
                                              PathPoint(22.996, -13.521, rotatingspeed),
                                              PathPoint(22.704, -14.014, rotatingspeed),
                                              PathPoint(22.382, -14.489, rotatingspeed),
                                              PathPoint(22.031, -14.941, rotatingspeed),
                                              PathPoint(21.652, -15.371, rotatingspeed),
                                              PathPoint(21.246, -15.777, rotatingspeed),
                                              PathPoint(20.816, -16.156, rotatingspeed),
                                              PathPoint(20.364, -16.507, rotatingspeed),
                                              PathPoint(19.889, -16.829, rotatingspeed),
                                              PathPoint(19.396, -17.121, rotatingspeed),
                                              PathPoint(18.885, -17.382, rotatingspeed),
                                              PathPoint(18.359, -17.609, rotatingspeed),
                                              PathPoint(17.820, -17.803, rotatingspeed),
                                              PathPoint(17.269, -17.963, rotatingspeed),
                                              PathPoint(16.710, -18.088, rotatingspeed),
                                              PathPoint(16.144, -18.178, rotatingspeed),
                                              PathPoint(15.573, -18.232, rotatingspeed),
                                              PathPoint(15.000, -18.250, rotatingspeed),
                                              PathPoint(14.427, -18.232, rotatingspeed),
                                              PathPoint(13.856, -18.178, rotatingspeed),
                                              PathPoint(13.290, -18.088, rotatingspeed),
                                              PathPoint(12.731, -17.963, rotatingspeed),
                                              PathPoint(12.180, -17.803, rotatingspeed),
                                              PathPoint(11.641, -17.609, rotatingspeed),
                                              PathPoint(11.115, -17.382, rotatingspeed),
                                              PathPoint(10.604, -17.121, rotatingspeed),
                                              PathPoint(10.111, -16.829, rotatingspeed),
                                              PathPoint(9.636, -16.507, rotatingspeed),
                                              PathPoint(9.184, -16.156, rotatingspeed),
                                              PathPoint(8.754, -15.777, rotatingspeed),
                                              PathPoint(8.348, -15.371, rotatingspeed),
                                              PathPoint(7.969, -14.941, rotatingspeed),
                                              PathPoint(7.618, -14.489, rotatingspeed),
                                              PathPoint(7.296, -14.014, rotatingspeed),
                                              PathPoint(7.004, -13.521, rotatingspeed),
                                              PathPoint(6.743, -13.010, rotatingspeed),
                                              PathPoint(6.516, -12.484, rotatingspeed),
                                              PathPoint(6.322, -11.945, rotatingspeed),
                                              PathPoint(6.162, -11.394, rotatingspeed),
                                              PathPoint(6.037, -10.835, rotatingspeed),
                                              PathPoint(5.947, -10.269, rotatingspeed),
                                              PathPoint(5.893, -9.698, rotatingspeed),
                                              PathPoint(5.875, -9.125, rotatingspeed),
                                              PathPoint(5.893, -8.552, rotatingspeed),
                                              PathPoint(5.947, -7.981, rotatingspeed),
                                              PathPoint(6.037, -7.415, rotatingspeed),
                                              PathPoint(6.162, -6.856, rotatingspeed),
                                              PathPoint(6.322, -6.305, rotatingspeed),
                                              PathPoint(6.516, -5.766, rotatingspeed),
                                              PathPoint(6.743, -5.240, rotatingspeed),
                                              PathPoint(7.004, -4.729, rotatingspeed),
                                              PathPoint(7.296, -4.236, rotatingspeed),
                                              PathPoint(7.618, -3.761, rotatingspeed),
                                              PathPoint(7.969, -3.309, rotatingspeed),
                                              PathPoint(8.348, -2.879, rotatingspeed),
                                              PathPoint(8.754, -2.473, rotatingspeed),
                                              PathPoint(9.184, -2.094, rotatingspeed),
                                              PathPoint(9.636, -1.743, rotatingspeed),
                                              PathPoint(10.111, -1.421, rotatingspeed),
                                              PathPoint(10.604, -1.129, rotatingspeed),
                                              PathPoint(11.115, -0.868, rotatingspeed),
                                              PathPoint(11.641, -0.641, rotatingspeed),
                                              PathPoint(12.180, -0.447, rotatingspeed),
                                              PathPoint(12.731, -0.287, rotatingspeed),
                                              PathPoint(13.290, -0.162, rotatingspeed),
                                              PathPoint(13.856, -0.072, rotatingspeed),
                                              PathPoint(14.427, -0.018, rotatingspeed),
                                              PathPoint(15.000, 0.000, rotatingspeed),
                                              PathPoint(15.573, 0.018, rotatingspeed),
                                              PathPoint(16.144, 0.072, rotatingspeed),
                                              PathPoint(16.710, 0.162, rotatingspeed),
                                              PathPoint(17.269, 0.287, rotatingspeed),
                                              PathPoint(17.820, 0.447, rotatingspeed),
                                              PathPoint(18.359, 0.641, rotatingspeed),
                                              PathPoint(18.885, 0.868, rotatingspeed),
                                              PathPoint(19.396, 1.129, rotatingspeed),
                                              PathPoint(19.889, 1.421, rotatingspeed),
                                              PathPoint(20.364, 1.743, rotatingspeed),
                                              PathPoint(20.816, 2.094, rotatingspeed),
                                              PathPoint(21.246, 2.473, rotatingspeed),
                                              PathPoint(21.652, 2.879, rotatingspeed),
                                              PathPoint(22.031, 3.309, rotatingspeed),
                                              PathPoint(22.382, 3.761, rotatingspeed),
                                              PathPoint(22.704, 4.236, rotatingspeed),
                                              PathPoint(22.996, 4.729, rotatingspeed),
                                              PathPoint(23.257, 5.240, rotatingspeed),
                                              PathPoint(23.484, 5.766, rotatingspeed),
                                              PathPoint(23.678, 6.305, rotatingspeed),
                                              PathPoint(23.838, 6.856, rotatingspeed),
                                              PathPoint(23.963, 7.415, rotatingspeed),
                                              PathPoint(24.053, 7.981, rotatingspeed),
                                              PathPoint(24.107, 8.552, rotatingspeed),
                                              PathPoint(24.125, 9.125, rotatingspeed),
                                              PathPoint(24.107, 9.698, rotatingspeed),
                                              PathPoint(24.053, 10.269, rotatingspeed),
                                              PathPoint(23.963, 10.835, rotatingspeed),
                                              PathPoint(23.838, 11.394, rotatingspeed),
                                              PathPoint(23.678, 11.945, rotatingspeed),
                                              PathPoint(23.484, 12.484, rotatingspeed),
                                              PathPoint(23.257, 13.010, rotatingspeed),
                                              PathPoint(22.996, 13.521, rotatingspeed),
                                              PathPoint(22.704, 14.014, rotatingspeed),
                                              PathPoint(22.382, 14.489, rotatingspeed),
                                              PathPoint(22.031, 14.941, rotatingspeed),
                                              PathPoint(21.652, 15.371, rotatingspeed),
                                              PathPoint(21.246, 15.777, rotatingspeed),
                                              PathPoint(20.816, 16.156, rotatingspeed),
                                              PathPoint(20.364, 16.507, rotatingspeed),
                                              PathPoint(19.889, 16.829, rotatingspeed),
                                              PathPoint(19.396, 17.121, rotatingspeed),
                                              PathPoint(18.885, 17.382, rotatingspeed),
                                              PathPoint(18.359, 17.609, rotatingspeed),
                                              PathPoint(17.820, 17.803, rotatingspeed),
                                              PathPoint(17.269, 17.963, rotatingspeed),
                                              PathPoint(16.710, 18.088, rotatingspeed),
                                              PathPoint(16.144, 18.178, rotatingspeed),
                                              PathPoint(15.573, 18.232, rotatingspeed),
                                              PathPoint(15.000, 18.250, rotatingspeed),
                                              PathPoint(14.427, 18.232, rotatingspeed),
                                              PathPoint(13.856, 18.178, rotatingspeed),
                                              PathPoint(13.290, 18.088, rotatingspeed),
                                              PathPoint(12.731, 17.963, rotatingspeed),
                                              PathPoint(12.180, 17.803, rotatingspeed),
                                              PathPoint(11.641, 17.609, rotatingspeed),
                                              PathPoint(11.115, 17.382, rotatingspeed),
                                              PathPoint(10.604, 17.121, rotatingspeed),
                                              PathPoint(10.111, 16.829, rotatingspeed),
                                              PathPoint(9.636, 16.507, rotatingspeed),
                                              PathPoint(9.184, 16.156, rotatingspeed),
                                              PathPoint(8.754, 15.777, rotatingspeed),
                                              PathPoint(8.348, 15.371, rotatingspeed),
                                              PathPoint(7.969, 14.941, rotatingspeed),
                                              PathPoint(7.618, 14.489, rotatingspeed),
                                              PathPoint(7.296, 14.014, rotatingspeed),
                                              PathPoint(7.004, 13.521, rotatingspeed),
                                              PathPoint(6.743, 13.010, rotatingspeed),
                                              PathPoint(6.516, 12.484, rotatingspeed),
                                              PathPoint(6.322, 11.945, rotatingspeed),
                                              PathPoint(6.162, 11.394, rotatingspeed),
                                              PathPoint(6.037, 10.835, rotatingspeed),
                                              PathPoint(5.947, 10.269, rotatingspeed),
                                              PathPoint(5.893, 9.698, rotatingspeed),
                                              PathPoint(5.875, 9.125, rotatingspeed),
                                              PathPoint(5.893, 8.552, rotatingspeed),
                                              PathPoint(5.947, 7.981, rotatingspeed),
                                              PathPoint(6.037, 7.415, rotatingspeed),
                                              PathPoint(6.162, 6.856, rotatingspeed),
                                              PathPoint(6.322, 6.305, rotatingspeed),
                                              PathPoint(6.516, 5.766, rotatingspeed),
                                              PathPoint(6.743, 5.240, rotatingspeed),
                                              PathPoint(7.004, 4.729, rotatingspeed),
                                              PathPoint(7.296, 4.236, rotatingspeed),
                                              PathPoint(7.618, 3.761, rotatingspeed),
                                              PathPoint(7.969, 3.309, rotatingspeed),
                                              PathPoint(8.348, 2.879, rotatingspeed),
                                              PathPoint(8.754, 2.473, rotatingspeed),
                                              PathPoint(9.184, 2.094, rotatingspeed),
                                              PathPoint(9.636, 1.743, rotatingspeed),
                                              PathPoint(10.111, 1.421, rotatingspeed),
                                              PathPoint(10.604, 1.129, rotatingspeed),
                                              PathPoint(11.115, 0.868, rotatingspeed),
                                              PathPoint(11.641, 0.641, rotatingspeed),
                                              PathPoint(12.180, 0.447, rotatingspeed),
                                              PathPoint(12.731, 0.287, rotatingspeed),
                                              PathPoint(13.290, 0.162, rotatingspeed),
                                              PathPoint(13.856, 0.072, rotatingspeed),
                                              PathPoint(14.427, 0.018, rotatingspeed),
                                              PathPoint(15.000, 0.000, rotatingspeed),
                                              PathPoint(15.573, 0.018, rotatingspeed),
                                              PathPoint(16.144, 0.072, rotatingspeed),
                                              PathPoint(16.710, 0.162, rotatingspeed),
                                              PathPoint(17.269, 0.287, rotatingspeed),
                                              PathPoint(17.820, 0.447, rotatingspeed),
                                              PathPoint(18.359, 0.641, rotatingspeed),
                                              PathPoint(18.885, 0.868, rotatingspeed),
                                              PathPoint(19.396, 1.129, rotatingspeed),
                                              PathPoint(19.889, 1.421, rotatingspeed),
                                              PathPoint(20.364, 1.743, rotatingspeed),
                                              PathPoint(20.816, 2.094, rotatingspeed),
                                              PathPoint(21.246, 2.473, rotatingspeed),
                                              PathPoint(21.652, 2.879, rotatingspeed),
                                              PathPoint(22.031, 3.309, rotatingspeed),
                                              PathPoint(22.382, 3.761, rotatingspeed),
                                              PathPoint(22.704, 4.236, rotatingspeed),
                                              PathPoint(22.996, 4.729, rotatingspeed),
                                              PathPoint(23.257, 5.240, rotatingspeed),
                                              PathPoint(23.484, 5.766, rotatingspeed),
                                              PathPoint(23.678, 6.305, rotatingspeed),
                                              PathPoint(23.838, 6.856, rotatingspeed),
                                              PathPoint(23.963, 7.415, rotatingspeed),
                                              PathPoint(24.053, 7.981, rotatingspeed),
                                              PathPoint(24.107, 8.552, rotatingspeed),
                                              PathPoint(24.125, 9.125, rotatingspeed),
                                              PathPoint(24.107, 9.698, rotatingspeed),
                                              PathPoint(24.053, 10.269, rotatingspeed),
                                              PathPoint(23.963, 10.835, rotatingspeed),
                                              PathPoint(23.838, 11.394, rotatingspeed),
                                              PathPoint(23.678, 11.945, rotatingspeed),
                                              PathPoint(23.484, 12.484, rotatingspeed),
                                              PathPoint(23.257, 13.010, rotatingspeed),
                                              PathPoint(22.996, 13.521, rotatingspeed),
                                              PathPoint(22.704, 14.014, rotatingspeed),
                                              PathPoint(22.382, 14.489, rotatingspeed),
                                              PathPoint(22.031, 14.941, rotatingspeed),
                                              PathPoint(21.652, 15.371, rotatingspeed),
                                              PathPoint(21.246, 15.777, rotatingspeed),
                                              PathPoint(20.816, 16.156, rotatingspeed),
                                              PathPoint(20.364, 16.507, rotatingspeed),
                                              PathPoint(19.889, 16.829, rotatingspeed),
                                              PathPoint(19.396, 17.121, rotatingspeed),
                                              PathPoint(18.885, 17.382, rotatingspeed),
                                              PathPoint(18.359, 17.609, rotatingspeed),
                                              PathPoint(17.820, 17.803, rotatingspeed),
                                              PathPoint(17.269, 17.963, rotatingspeed),
                                              PathPoint(16.710, 18.088, rotatingspeed),
                                              PathPoint(16.144, 18.178, rotatingspeed),
                                              PathPoint(15.573, 18.232, rotatingspeed),
                                              PathPoint(15.000, 18.250, rotatingspeed),
                                              PathPoint(14.427, 18.232, rotatingspeed),
                                              PathPoint(13.856, 18.178, rotatingspeed),
                                              PathPoint(13.290, 18.088, rotatingspeed),
                                              PathPoint(12.731, 17.963, rotatingspeed),
                                              PathPoint(12.180, 17.803, rotatingspeed),
                                              PathPoint(11.641, 17.609, rotatingspeed),
                                              PathPoint(11.115, 17.382, rotatingspeed),
                                              PathPoint(10.604, 17.121, rotatingspeed),
                                              PathPoint(10.111, 16.829, rotatingspeed),
                                              PathPoint(9.636, 16.507, rotatingspeed),
                                              PathPoint(9.184, 16.156, rotatingspeed),
                                              PathPoint(8.754, 15.777, rotatingspeed),
                                              PathPoint(8.348, 15.371, rotatingspeed),
                                              PathPoint(7.969, 14.941, rotatingspeed),
                                              PathPoint(7.618, 14.489, rotatingspeed),
                                              PathPoint(7.296, 14.014, rotatingspeed),
                                              PathPoint(7.004, 13.521, rotatingspeed),
                                              PathPoint(6.743, 13.010, rotatingspeed),
                                              PathPoint(6.516, 12.484, rotatingspeed),
                                              PathPoint(6.322, 11.945, rotatingspeed),
                                              PathPoint(6.162, 11.394, rotatingspeed),
                                              PathPoint(6.037, 10.835, rotatingspeed),
                                              PathPoint(5.947, 10.269, rotatingspeed),
                                              PathPoint(5.893, 9.698, rotatingspeed),
                                              PathPoint(5.875, 9.125, rotatingspeed),
                                              PathPoint(5.893, 8.552, rotatingspeed),
                                              PathPoint(5.947, 7.981, rotatingspeed),
                                              PathPoint(6.037, 7.415, rotatingspeed),
                                              PathPoint(6.162, 6.856, rotatingspeed),
                                              PathPoint(6.322, 6.305, rotatingspeed),
                                              PathPoint(6.516, 5.766, rotatingspeed),
                                              PathPoint(6.743, 5.240, rotatingspeed),
                                              PathPoint(7.004, 4.729, rotatingspeed),
                                              PathPoint(7.296, 4.236, rotatingspeed),
                                              PathPoint(7.618, 3.761, rotatingspeed),
                                              PathPoint(7.969, 3.309, rotatingspeed),
                                              PathPoint(8.348, 2.879, rotatingspeed),
                                              PathPoint(8.754, 2.473, rotatingspeed),
                                              PathPoint(9.184, 2.094, rotatingspeed),
                                              PathPoint(9.636, 1.743, rotatingspeed),
                                              PathPoint(10.111, 1.421, rotatingspeed),
                                              PathPoint(10.604, 1.129, rotatingspeed),
                                              PathPoint(11.115, 0.868, rotatingspeed),
                                              PathPoint(11.641, 0.641, rotatingspeed),
                                              PathPoint(12.180, 0.447, rotatingspeed),
                                              PathPoint(12.731, 0.287, rotatingspeed),
                                              PathPoint(13.290, 0.162, rotatingspeed),
                                              PathPoint(13.856, 0.072, rotatingspeed),
                                              PathPoint(14.427, 0.018, rotatingspeed),
                                              PathPoint(15.000, 0, exitspeed),
                                              PathPoint(15.500, 0, exitspeed),
                                              PathPoint(16.000, 0, exitspeed),
                                              PathPoint(16.500, 0, exitspeed),
                                              PathPoint(17.000, 0, exitspeed),
                                              PathPoint(17.500, 0, exitspeed),
                                              PathPoint(18.000, 0, exitspeed),
                                              PathPoint(18.500, 0, exitspeed),
                                              PathPoint(19.000, 0, exitspeed),
                                              PathPoint(19.500, 0, exitspeed),
                                              PathPoint(20.000, 0, exitspeed),
                                              PathPoint(20.500, 0, exitspeed),
                                              PathPoint(21.000, 0, exitspeed),
                                              PathPoint(21.500, 0, exitspeed),
                                              PathPoint(22.000, 0, exitspeed),
                                              PathPoint(22.500, 0, exitspeed),
                                              PathPoint(23.000, 0, exitspeed),
                                              PathPoint(23.500, 0, exitspeed),
                                              PathPoint(24.000, 0, exitspeed),
                                              PathPoint(24.500, 0, exitspeed),
                                              PathPoint(25.000, 0, exitspeed),
                                              PathPoint(25.500, 0, exitspeed),
                                              PathPoint(26.000, 0, exitspeed),
                                              PathPoint(26.500, 0, exitspeed),
                                              PathPoint(27.000, 0, exitspeed),
                                              PathPoint(27.500, 0, 0),
                                              PathPoint(28.000, 0, 0),
                                              PathPoint(28.500, 0, 0),
                                              PathPoint(29.000, 0, 0),
                                              PathPoint(29.500, 0, 0),
                                              PathPoint(30.000, 0, 0),
                                              PathPoint(30.500, 0, 0),
                                              PathPoint(31.000, 0, 0),
                                              PathPoint(31.500, 0, 0),
                                              PathPoint(32.000, 0, 0),
                                              PathPoint(32.500, 0, 0),
                                              PathPoint(33.000, 0, 0),
                                              PathPoint(33.500, 0, 0),
                                              PathPoint(34.000, 0, 0),
                                              PathPoint(34.500, 0, 0),
                                              PathPoint(35.000, 0, 0),
                                              PathPoint(35.500, 0, 0),
                                              PathPoint(36.000, 0, 0),
                                              PathPoint(36.500, 0, 0),
                                              PathPoint(37.000, 0, 0),
                                              PathPoint(37.500, 0, 0),
                                              PathPoint(38.000, 0, 0),
                                              PathPoint(38.500, 0, 0),
                                              PathPoint(39.000, 0, 0),
                                              PathPoint(39.500, 0, 0),
                                              PathPoint(40.000, 0, 0),
                                              PathPoint(40.500, 0, 0),
                                              PathPoint(41.000, 0, 0),
                                              PathPoint(41.500, 0, 0),
                                              PathPoint(42.000, 0, 0),
                                              PathPoint(42.500, 0, 0),
                                              PathPoint(43.000, 0, 0),
                                              PathPoint(43.500, 0, 0),
                                              PathPoint(44.000, 0, 0),
                                              PathPoint(44.500, 0, 0)};

    if (cone_array.size() < 4) {
      throw std::runtime_error("Insufficient cones to calculate path");  // --------------------
    }
    // sort the cones by distance
    std::sort(cone_array.begin(), cone_array.end(), [&pose](Cone &a, Cone &b) {
      return sqrt(pow((a.position.x - pose.position.x), 2) +
                  pow((a.position.y - pose.position.y), 2)) <
             sqrt(pow((b.position.x - pose.position.x), 2) +
                  pow((b.position.y - pose.position.y), 2));
    });

    // get the middle point between the two closest points
    PathPoint middle_closest =
        PathPoint((cone_array[0].position.x + cone_array[1].position.x) / 2,
                  (cone_array[0].position.y + cone_array[1].position.y) / 2, 0);

    // get the middle point between the two farthest points
    PathPoint middle_farthest =
        PathPoint((cone_array[2].position.x + cone_array[3].position.x) / 2,
                  (cone_array[2].position.y + cone_array[3].position.y) / 2, 0);

    // Find the slope of the line between the two middle points
    double dx = middle_farthest.position.x - middle_closest.position.x;  // --------------------
    double dy = middle_farthest.position.y - middle_closest.position.y;  // --------------------
    double slope = (dx != 0) ? (dy / dx) : 10000000000;                  // --------------------

    // calculate the angle
    double angle = atan(slope);

    // rotate all the points by the angle and add the origin point
    for (auto &point : hardcoded_path_) {
      double x = point.position.x;
      double y = point.position.y;
      point.position.x = x * cos(angle) - y * sin(angle) + middle_closest.position.x;
      point.position.y = x * sin(angle) + y * cos(angle) + middle_closest.position.y;
    }

    predefined_path_ = hardcoded_path_;
    path_orientation_corrected_ = true;
  }

  while (!predefined_path_.empty() &&
         pose.position.euclidean_distance(predefined_path_[0].position) < 1) {
    predefined_path_.erase(predefined_path_.begin());
  }

  // set it as the final path
  return std::vector<PathPoint>(predefined_path_.begin(), predefined_path_.begin() + 70);
}
