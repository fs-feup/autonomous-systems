#include <cstdio>

#include "loc_map/lm_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  Localization *vehicle_localization = new Localization();
  Map *track_map = new Map();

  track_map->map.insert({{1, 2}, colors::red});

  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LMPublisher>(vehicle_localization, track_map));
  rclcpp::shutdown();

  return 0;
}
