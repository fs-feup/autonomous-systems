#include <string>
#include <filesystem>
#include <vector>
#include "../planning/track.hpp"
#include "./position.hpp"
#include "rclcpp/rclcpp.hpp"

Track* read_track_file(const std::string& filename);

void write_path_file(const std::string& filename, std::vector<Position*> path);