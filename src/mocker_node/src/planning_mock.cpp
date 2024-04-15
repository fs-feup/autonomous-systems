#include "include/planning_mock.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <istream>
#include <sstream>
#include <fstream>
#include <string>


custom_interfaces::msg::PathPointArray gtruth_fromfile(std::istream &in) {
    std::string x, y, velocity;
    custom_interfaces::msg::PathPointArray gtruth_mock;

    std::string line;
    in >> line; //ignore first line
    while (std::getline(in, line)) {
        std::stringstream iss(line);
        if (getline(iss, x, ',') &&
            getline(iss, y, ',') &&
            getline(iss, velocity, ',')) {
            try {
                custom_interfaces::msg::PathPoint custom_point;
                custom_point.x = std::stod(x);
                custom_point.y = std::stod(y);
                custom_point.v = std::stod(velocity);
                gtruth_mock.pathpoint_array.push_back(custom_point);
            } catch (const std::invalid_argument& e) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                "Invalid argument encountered while converting to double: %c \n", e.what());
            } catch (const std::out_of_range& e) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                "Out of range exception encountered while converting to double: %c \n", e.what());
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
            "Unexpected format in line: %s \n", line.c_str());
        }
    }

    return gtruth_mock;
}

std::istream& openFileAsStream(const std::string& filename) {
    static std::ifstream file;
    file.open(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
        "Unable to open file %s \n", filename.c_str());
    }
    return file;
}
