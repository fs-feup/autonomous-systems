#include "planning/colorpoint.hpp"

void Colorpoint::extract_cones(std::vector<Colorpoint>& colorpoints,
                               std::vector<Cone>& yellow_cones, std::vector<Cone>& blue_cones) {
  // Need at least 2 points to determine path direction
  if (colorpoints.size() < 2) {
    return;
  }
  std::vector<Cone> yellow_cones1;
  std::vector<Cone> blue_cones1;
  // Process all colorpoints except the last one
  for (size_t i = 0; i < colorpoints.size() - 1; i++) {
    // Check if cones are already colored correctly,if not use cross product to determine colors
    // if (!is_colored_right(colorpoints[i], yellow_cones, blue_cones)) {
    color_cones(colorpoints[i], colorpoints[i + 1], yellow_cones1, blue_cones1);
    // }
  }

  // Handle the last point using cone matching from second-to-last point
  color_last_point(colorpoints, yellow_cones1, blue_cones1);
  yellow_cones = yellow_cones1;
  blue_cones = blue_cones1;
}

bool Colorpoint::is_colored_right(const Colorpoint& colorpoint, std::vector<Cone>& yellow_cones,
                                  std::vector<Cone>& blue_cones) {
  if (colorpoint.cone1.color == Color::YELLOW && colorpoint.cone2.color == Color::BLUE) {
    yellow_cones.push_back(colorpoint.cone1);
    blue_cones.push_back(colorpoint.cone2);
    return true;
  }

  if (colorpoint.cone1.color == Color::BLUE && colorpoint.cone2.color == Color::YELLOW) {
    blue_cones.push_back(colorpoint.cone1);
    yellow_cones.push_back(colorpoint.cone2);
    return true;
  }

  return false;
}

void Colorpoint::color_cones(Colorpoint& colorpoint, const Colorpoint& next_colorpoint,
                             std::vector<Cone>& yellow_cones, std::vector<Cone>& blue_cones) {
  // Calculate path direction vector from current point to next point
  double path_dx = next_colorpoint.point.x() - colorpoint.point.x();
  double path_dy = next_colorpoint.point.y() - colorpoint.point.y();

  // Calculate vector from path point to cone1
  double cone1_dx = colorpoint.cone1.position.x - colorpoint.point.x();
  double cone1_dy = colorpoint.cone1.position.y - colorpoint.point.y();

  // Calculate vector from path point to cone2
  double cone2_dx = colorpoint.cone2.position.x - colorpoint.point.x();
  double cone2_dy = colorpoint.cone2.position.y - colorpoint.point.y();

  // Calculate cross products to determine which side each cone is on
  double cross_product1 = path_dx * cone1_dy - path_dy * cone1_dx;
  double cross_product2 = path_dx * cone2_dy - path_dy * cone2_dx;

  // Positive value of cross product means cone is on the left and negative on the right
  if (cross_product1 > cross_product2) {
    colorpoint.cone1.color = Color::BLUE;
    colorpoint.cone2.color = Color::YELLOW;
    blue_cones.push_back(colorpoint.cone1);
    yellow_cones.push_back(colorpoint.cone2);
  } else {
    colorpoint.cone1.color = Color::YELLOW;
    colorpoint.cone2.color = Color::BLUE;
    yellow_cones.push_back(colorpoint.cone1);
    blue_cones.push_back(colorpoint.cone2);
  }
}

void Colorpoint::add_cones_by_reference(const Cone& reference_cone, const Cone& matching_cone,
                                        const Cone& other_cone, std::vector<Cone>& yellow_cones,
                                        std::vector<Cone>& blue_cones) {
  if (reference_cone.color == Color::BLUE) {
    blue_cones.push_back(matching_cone);
    yellow_cones.push_back(other_cone);
  } else {
    yellow_cones.push_back(matching_cone);
    blue_cones.push_back(other_cone);
  }
}

void Colorpoint::color_last_point(std::vector<Colorpoint>& colorpoints,
                                  std::vector<Cone>& yellow_cones, std::vector<Cone>& blue_cones) {
  const Colorpoint& second_to_last = colorpoints[colorpoints.size() - 2];
  const Colorpoint& last = colorpoints.back();

  // Check which cone from second-to-last matches which cone in last
  if (second_to_last.cone1 == last.cone1) {
    add_cones_by_reference(second_to_last.cone1, last.cone1, last.cone2, yellow_cones, blue_cones);
  } else if (second_to_last.cone1 == last.cone2) {
    add_cones_by_reference(second_to_last.cone1, last.cone2, last.cone1, yellow_cones, blue_cones);
  } else if (second_to_last.cone2 == last.cone1) {
    add_cones_by_reference(second_to_last.cone2, last.cone1, last.cone2, yellow_cones, blue_cones);
  } else if (second_to_last.cone2 == last.cone2) {
    add_cones_by_reference(second_to_last.cone2, last.cone2, last.cone1, yellow_cones, blue_cones);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("planning"),
                "The last cone does not match with any previous cone.");
  }
}
