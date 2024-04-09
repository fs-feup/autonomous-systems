#ifndef MOCKER_NODE_INCLUDE_PLANNING_MOCK_HPP
#define MOCKER_NODE_INCLUDE_PLANNING_MOCK_HPP

// #include "adapter/adapter.hpp"

#include <vector>
#include <string>
#include "custom_interfaces/msg/path_point.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"

/**
 * @brief read ground truth information from file and place it in an array of PathPoint objects
 * 
 * @param gtruth_file_path file to the csv file containing the ground truth
 * @return custom_interfaces::msg::PathPointArray vector with the path points in the ground truth
 */
custom_interfaces::msg::PathPointArray gtruth_fromfile(std::string gtruth_file_path);


#endif