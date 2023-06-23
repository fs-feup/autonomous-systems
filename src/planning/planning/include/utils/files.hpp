#include <string>
#include <filesystem>
#include "../planning/track.hpp"
#include "./position.hpp"
#include "rclcpp/rclcpp.hpp"

Track* read_track_file(std::string filename);

void write_path_file(std::string filename, std::vector<Position*> path);