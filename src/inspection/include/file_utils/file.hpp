#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <string>
#include <vector>
#include <cstring>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

/**
 * @brief open file and return
 * 
 * @param filename path to the file from repository
 * @return std::ifstream open file
 */
std::ifstream openFileRead(const std::string &filename);
