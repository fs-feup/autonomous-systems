#include "planning/colorpoint.hpp"

void Colorpoint::extract_cones(std::vector<Colorpoint>& colorpoints,
                               std::vector<Cone>& yellow_cones, std::vector<Cone>& blue_cones) {
  // Need at least 2 points to determine path direction
  if (colorpoints.size() < 2) {
    return;
  }

  // Process all colorpoints except the last one
  for (size_t i = 0; i < colorpoints.size() - 1; i++) {
    // Check if cones are already colored correctly,if not use cross product to determine colors
    if (!is_colored_right(colorpoints[i], yellow_cones, blue_cones)) {
      color_cones(colorpoints[i], colorpoints[i + 1], yellow_cones, blue_cones);
    }
  }

  // Handle the last point using cone matching from second-to-last point
  color_last_point(colorpoints, yellow_cones, blue_cones);
}

void Colorpoint::add_cone(const Cone& cone, std::vector<Cone>& cones) {
  // Only add if it's the first cone or different from the previous one
  if (cones.empty() || !(cones[cones.size() - 2] == cone)) {
    cones.push_back(cone);
  }
}

bool Colorpoint::is_colored_right(Colorpoint& colorpoint, std::vector<Cone>& yellow_cones,
                                  std::vector<Cone>& blue_cones) {
  if (colorpoint.cone1.color == Color::YELLOW && colorpoint.cone2.color == Color::BLUE) {
    add_cone(colorpoint.cone1, yellow_cones);
    add_cone(colorpoint.cone2, blue_cones);
    return true;
  }

  if (colorpoint.cone1.color == Color::BLUE && colorpoint.cone2.color == Color::YELLOW) {
    add_cone(colorpoint.cone1, blue_cones);
    add_cone(colorpoint.cone2, yellow_cones);
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
    add_cone(colorpoint.cone1, blue_cones);
    add_cone(colorpoint.cone2, yellow_cones);
  } else {
    colorpoint.cone1.color = Color::YELLOW;
    colorpoint.cone2.color = Color::BLUE;
    add_cone(colorpoint.cone1, yellow_cones);
    add_cone(colorpoint.cone2, blue_cones);
  }
}

void Colorpoint::add_cones_by_reference(const Cone& reference_cone, Cone& matching_cone,
                                        Cone& other_cone, std::vector<Cone>& yellow_cones,
                                        std::vector<Cone>& blue_cones) {
  if (reference_cone.color == Color::BLUE) {
    add_cone(matching_cone, blue_cones);
    add_cone(other_cone, yellow_cones);
  } else {
    add_cone(matching_cone, yellow_cones);
    add_cone(other_cone, blue_cones);
  }
}

void Colorpoint::color_last_point(std::vector<Colorpoint>& colorpoints,
                                  std::vector<Cone>& yellow_cones, std::vector<Cone>& blue_cones) {
  const Colorpoint& second_to_last = colorpoints[colorpoints.size() - 2];
  Colorpoint& last = colorpoints.back();

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
