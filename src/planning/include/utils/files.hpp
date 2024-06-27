#ifndef SRC_PLANNING_PLANNING_INCLUDE_UTILS_FILES_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_UTILS_FILES_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "common_lib/structures/path_point.hpp"

using PathPoint = common_lib::structures::PathPoint;

std::vector<PathPoint> read_path_file(const std::string &filename);

std::ofstream openWriteFile(const std::string &filename, const std::string &header = "");

std::ifstream openReadFile(const std::string &filename);

// Track* read_track_file(const std::string& filename);

// void write_path_file(const std::string& filename, std::vector<PathPoint*>
// path);

#endif  // SRC_PLANNING_PLANNING_INCLUDE_UTILS_FILES_HPP_