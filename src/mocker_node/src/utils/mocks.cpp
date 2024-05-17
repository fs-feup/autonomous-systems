#include "utils/mocks.hpp"

#include <fstream>
#include <iostream>
#include <istream>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

custom_interfaces::msg::PathPointArray planning_gtruth_fromfile(std::istream& in) {
  std::string x;
  std::string y;
  std::string velocity;
  custom_interfaces::msg::PathPointArray gtruth_mock;

  std::string line;
  std::getline(in, line);  // ignore first line
  while (std::getline(in, line)) {
    std::stringstream iss(line);
    if (getline(iss, x, ',') && getline(iss, y, ',') && getline(iss, velocity, ',')) {
      try {
        custom_interfaces::msg::PathPoint custom_point;
        custom_point.x = std::stod(x);
        custom_point.y = std::stod(y);
        custom_point.v = std::stod(velocity);
        gtruth_mock.pathpoint_array.push_back(custom_point);
      } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Invalid argument encountered while converting to double: %s \n", e.what());
      } catch (const std::out_of_range& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Out of range exception encountered while converting to double: %s \n",
                     e.what());
      }
    } else if (!line.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unexpected format in line with content: %s \n",
                   line.c_str());
    }
  }

  return gtruth_mock;
}

custom_interfaces::msg::ConeArray se_gtruth_fromfile(std::istream& in) {
  std::string x;
  std::string y;
  std::string color;
  custom_interfaces::msg::ConeArray gtruth_mock;

  std::string line;
  std::getline(in, line);  // ignore first line
  while (std::getline(in, line)) {
    std::stringstream iss(line);
    if (getline(iss, x, ',') && getline(iss, y, ',') && getline(iss, color, ',')) {
      try {
        custom_interfaces::msg::Cone cone;
        cone.position.x = std::stod(x);
        cone.position.y = std::stod(y);
        cone.color =
            color + "_cone";  // 1 is extra space to remove first char being empty
        gtruth_mock.cone_array.push_back(cone);
      } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Invalid argument encountered while converting to double: %s \n", e.what());
      } catch (const std::out_of_range& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Out of range exception encountered while converting to double: %s \n",
                     e.what());
      }
    } else if (!line.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unexpected format in line with content: %s \n",
                   line.c_str());
    }
  }
  in.clear();
  in.seekg(0);
  return gtruth_mock;
}

std::istream& open_file_as_stream(const std::string& filename) {
  static std::ifstream file;
  if (file.is_open()) file.close();

  file.open(filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to open file %s \n", filename.c_str());
  }
  return file;
}
