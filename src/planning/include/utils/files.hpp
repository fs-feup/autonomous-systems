#ifndef SRC_PLANNING_PLANNING_INCLUDE_UTILS_FILES_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_UTILS_FILES_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <string>
#include <vector>

#include "../planning/track.hpp"
#include "./pathpoint.hpp"
#include "rclcpp/rclcpp.hpp"

std::vector<PathPoint *> read_path_file(const std::string &filename);

std::ofstream openWriteFile(const std::string &filename);

std::ifstream openReadFile(const std::string &filename);

// Track* read_track_file(const std::string& filename);

// void write_path_file(const std::string& filename, std::vector<PathPoint*>
// path);

#endif  // SRC_PLANNING_PLANNING_INCLUDE_UTILS_FILES_HPP_