#include "utils/files.hpp"

#include <filesystem>
#include <string>
#include <vector>

std::vector<PathPoint> read_path_file(const std::string &filename) {
  std::string filePrefix = ament_index_cpp::get_package_share_directory("planning");
  std::string filepath = filePrefix + "/planning/planning" + filename;
  std::ifstream path_file(filepath);
  std::vector<PathPoint> path;
  float x, y;
  while (path_file >> x >> y) {
    PathPoint pathpoint = PathPoint(x, y);
    path.push_back(pathpoint);
  }
  path_file.close();
  return path;
}

std::ofstream openWriteFile(const std::string &filename, const std::string &header) {
  std::string filePrefix = ament_index_cpp::get_package_share_directory("planning");
  filePrefix = filePrefix.substr(0, filePrefix.find("install"));
  std::string logger_variable = filePrefix + filename;

  bool fileExists = std::filesystem::exists(filePrefix + filename);

  std::ofstream file(filePrefix + filename, std::ios::app);

  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR opening file: %s\n", logger_variable.c_str());
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Successfully opened %s \n",
                 logger_variable.c_str());
  }

  if (!fileExists) {
    file << header << "\n";
  }

  return file;
}

std::ifstream openReadFile(const std::string &filename) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Start openReadFile");
  std::string filePrefix = ament_index_cpp::get_package_share_directory("planning");
  filePrefix = filePrefix.substr(0, filePrefix.find("install"));
  std::string logger_variable = filePrefix + filename;
  std::ifstream file(filePrefix + filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR opening file: %s\n", logger_variable.c_str());
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Successfully opened %s \n",
                 logger_variable.c_str());
  }
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "End openReadFile");
  return file;
}

