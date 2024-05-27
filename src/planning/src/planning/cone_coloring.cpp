#include "planning/cone_coloring.hpp"

#include <algorithm>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

ConeColoring::ConeColoring(double gain_angle, double gain_distance, double gain_ncones,
                           double exponent_1, double exponent_2, double cost_max)
    : angle_gain(gain_angle),
      distance_gain(gain_distance),
      ncones_gain(gain_ncones),
      exponent1(exponent_1),
      exponent2(exponent_2),
      max_cost(cost_max) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
               "Cone Coloring: angle gain is %f; distance_gain is %f", this->angle_gain,
               this->distance_gain);
}

AngleNorm ConeColoring::angle_and_norm(const Cone *possible_next_cone, TrackSide side) const {
  const std::vector<Cone *> &current_cones = (bool)side ? current_left_cones : current_right_cones;
  auto n = (int)current_cones.size();
  double x_last = current_cones[n - 1]->getX() - current_cones[n - 2]->getX();
  double y_last = current_cones[n - 1]->getY() - current_cones[n - 2]->getY();
  double norm2 = sqrt(x_last * x_last + y_last * y_last);
  double x_next = possible_next_cone->getX() - current_cones[n - 1]->getX();
  double y_next = possible_next_cone->getY() - current_cones[n - 1]->getY();
  double new_norm = sqrt(x_next * x_next + y_next * y_next);
  double dot_product = x_last * x_next + y_last * y_next;
  return {acos(dot_product / (new_norm * norm2)), new_norm};
}

bool ConeColoring::cone_is_in_side(const Cone *c, TrackSide side) const {
  const std::vector<Cone *> &current_cones = (bool)side ? current_left_cones : current_right_cones;
  auto it = std::find_if(current_cones.begin(), current_cones.end(), [&c](const Cone *cone) {
    return cone->getX() == c->getX() && cone->getY() == c->getY();
  });
  return it != current_cones.end();
}

double ConeColoring::distance_to_side(Pose initial_car_pose, TrackSide side, double x,
                                      double y) const {
  if ((bool)side)
    return fabs(y - (initial_car_pose.position.y + 2 * cos(initial_car_pose.orientation)) -
                tan(initial_car_pose.orientation) *
                    (x - (initial_car_pose.position.x - 2 * sin(initial_car_pose.orientation))));
  else
    return fabs(y - (initial_car_pose.position.y - 2 * cos(initial_car_pose.orientation)) -
                tan(initial_car_pose.orientation) *
                    (x - (initial_car_pose.position.x + 2 * sin(initial_car_pose.orientation))));
}

Cone ConeColoring::get_initial_cone(const std::vector<Cone *> &candidates, Pose initial_car_pose,
                                    TrackSide side) const {
  double minimum_cost = MAXFLOAT;
  Cone initial_cone(1 - (int)side, 0, 0);  // id must be 0 for left side and 1 for right side
  for (const Cone *c : candidates) {
    double distance_to_car = sqrt(pow(initial_car_pose.position.x - c->getX(), 2) +
                                  pow(initial_car_pose.position.y - c->getY(), 2));
    double cost = distance_to_car * distance_to_side(initial_car_pose, side, c->getX(), c->getY());
    if (cost < minimum_cost) {
      minimum_cost = cost;
      initial_cone.setX(c->getX());
      initial_cone.setY(c->getY());
    }
  }
  return initial_cone;
}

std::vector<Cone *> ConeColoring::filter_cones_by_distance(const std::vector<Cone *> &input_cones,
                                                           Position position, double radius) const {
  std::vector<Cone *> filtered_cones;
  double radius_squared = radius * radius;
  for (Cone *c : input_cones) {
    double dx = position.x - c->getX();
    double dy = position.y - c->getY();
    double squared_distance = dx * dx + dy * dy;
    if (squared_distance <= radius_squared) filtered_cones.push_back(c);
  }
  return filtered_cones;
}

void ConeColoring::place_initial_cones(const std::vector<Cone *> &input_cones,
                                       Pose initial_car_pose) {
  // filter cones that are farther away than 5 meters
  std::vector<Cone *> candidates =
      filter_cones_by_distance(input_cones, initial_car_pose.position, 5);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Car position is : (%f,%f)",
               initial_car_pose.position.x, initial_car_pose.position.y);

  // get the initial cone for the left side
  this->initial_cone_left =
      std::make_shared<Cone>(get_initial_cone(candidates, initial_car_pose, TrackSide::LEFT));

  // get the initial cone for the right side
  this->initial_cone_right =
      std::make_shared<Cone>(get_initial_cone(candidates, initial_car_pose, TrackSide::RIGHT));

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Initial cones are at (%f, %f) and (%f, %f)",
               initial_cone_left->getX(), initial_cone_left->getY(), initial_cone_right->getX(),
               initial_cone_right->getY());

  // Create virtual cones to be added at the beginning of the vector. They must be dismissed later.
  virtual_left_cone = std::make_shared<Cone>(
      -2, initial_cone_left->getX() - 2 * (float)cos(initial_car_pose.orientation),
      initial_cone_left->getY() - 2 * (float)sin(initial_car_pose.orientation));
  virtual_right_cone = std::make_shared<Cone>(
      -1, initial_cone_right->getX() - 2 * (float)cos(initial_car_pose.orientation),
      initial_cone_right->getY() - 2 * (float)sin(initial_car_pose.orientation));

  current_left_cones.push_back(virtual_left_cone.get());
  current_right_cones.push_back(virtual_right_cone.get());
  current_left_cones.push_back(initial_cone_left.get());
  current_right_cones.push_back(initial_cone_right.get());
}

double ConeColoring::cost(const Cone *possible_next_cone, int n, TrackSide side) const {
  const std::vector<Cone *> &current_cones = (bool)side ? current_left_cones : current_right_cones;
  if (current_cones.size() < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Not enough cones to calculate cost when coloring cones");
    throw std::runtime_error("Not enough cones to calculate cost in Cone Coloring");
  }
  AngleNorm angle_norm;
  angle_norm = angle_and_norm(possible_next_cone, side);
  double cost = pow(distance_gain * angle_norm.norm, exponent1) +
                pow(angle_gain * angle_norm.angle, exponent2) +
                ncones_gain * (double)current_cones.size() / (float)n;
  return cost;
}

std::shared_ptr<std::vector<std::pair<Cone *, bool>>> ConeColoring::make_unvisited_cones(
    const std::vector<Cone *> &cones) const {
  auto cone_pairs = std::make_shared<std::vector<std::pair<Cone *, bool>>>();
  for (Cone *cone : cones) {
    auto cone_pair = std::make_pair(
        cone, cone_is_in_side(cone, TrackSide::LEFT) || cone_is_in_side(cone, TrackSide::RIGHT));
    cone_pairs->push_back(cone_pair);
  }
  return cone_pairs;
}

bool ConeColoring::place_next_cone(
    std::shared_ptr<std::vector<std::pair<Cone *, bool>>> visited_cones, double distance_threshold,
    int n, TrackSide side) {
  double minimum_cost = MAXFLOAT;

  std::vector<Cone *> &current_cones = (bool)side ? current_left_cones : current_right_cones;

  const Cone *last_cone = current_cones.back();

  const Cone *next_cone = (*visited_cones)[0].first;

  std::pair<Cone *, bool> *best_pair = &(*visited_cones)[0];

  double squared_distance_threshold = distance_threshold * distance_threshold;

  for (std::pair<Cone *, bool> &p : *visited_cones) {
    if (p.second) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Skipping visited cone at : %f , %f",
                   p.first->getX(), p.first->getY());
      continue;
    }  // skip visited cones
    double cost = ConeColoring::cost(p.first, n, side);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Cost is %f for cone at (%f,%f)", cost,
                 p.first->getX(), p.first->getY());
    if (cost < minimum_cost &&
        last_cone->squared_distance_to(p.first) < squared_distance_threshold && cost < max_cost) {
      minimum_cost = cost;
      next_cone = p.first;
      best_pair = &p;
    }
  }
  if (minimum_cost == MAXFLOAT) return false;
  auto selected_cone =
      new Cone(2 * (int)current_cones.size() - 1 - int(side), next_cone->getX(), next_cone->getY());
  current_cones.push_back(selected_cone);
  best_pair->second = true;
  return true;
}

void ConeColoring::color_cones(const std::vector<Cone *> &input_cones, Pose initial_car_pose,
                               double distance_threshold) {
  place_initial_cones(input_cones, initial_car_pose);
  auto visited_cones = make_unvisited_cones(input_cones);

  while (
      place_next_cone(visited_cones, distance_threshold, (int)input_cones.size(), TrackSide::LEFT))
    ;
  while (
      place_next_cone(visited_cones, distance_threshold, (int)input_cones.size(), TrackSide::RIGHT))
    ;
}
