#include "include/file_utils/file.hpp"

std::ifstream openFileRead(const std::string &filename) {
  std::string filePrefix = ament_index_cpp::get_package_share_directory("inspection");
  filePrefix = filePrefix.substr(0, filePrefix.find("install"));
  std::string logger_variable = filePrefix + filename;
  std::ifstream file(filePrefix + filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR opening file: %s\n", logger_variable.c_str());
  } else {RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
   "Successfully opened %s \n", logger_variable.c_str());}
  return file;
}