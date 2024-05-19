#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

#include <fstream>
#include <sstream>

#include "custom_interfaces/msg/path_point_array.hpp"
/**
 * @brief Reads track files and creates a PathPointArray message
 */

custom_interfaces::msg::PathPointArray create_path_msg(std::string track_file);


#endif  // TEST_UTILS_HPP_HPP_