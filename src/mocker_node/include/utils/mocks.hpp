#ifndef MOCKER_NODE_INCLUDE_PLANNING_MOCK_HPP
#define MOCKER_NODE_INCLUDE_PLANNING_MOCK_HPP

// #include "adapter/adapter.hpp"

#include <string>
#include <vector>

#include "custom_interfaces/msg/path_point.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/cone.hpp"
#include "custom_interfaces/msg/cone_array.hpp"

/**
 * @brief read planning ground truth information from stream and place it in an array of PathPoint objects
 *
 * @param  in input stream to extract the PathPoints from
 * @return custom_interfaces::msg::PathPointArray vector with the path points in the ground truth
 */
custom_interfaces::msg::PathPointArray planning_gtruth_fromfile(std::istream& in);

/**
 * @brief read state estimation ground truth information from stream and place it in an array of Cone objects
 *
 * @param  in input stream to extract the Cones
 * @return custom_interfaces::msg::ConeArray vector with the cones in the ground truth
 */
custom_interfaces::msg::ConeArray se_gtruth_fromfile(std::istream& in);

/**
 * @brief recieve path to a file and return it as an input stream
 *
 * @param filename path to the file
 * @return std::istream& open file
 */
std::istream& openFileAsStream(const std::string& filename);

#endif