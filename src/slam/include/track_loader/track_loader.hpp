#ifndef TRACKLOADER_HPP
#define TRACKLOADER_HPP

#include <Eigen/Dense>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

using namespace YAML;

/**
 * @brief Get the child node from a parent node by tag.
 *
 * @param parentNode parent node from which to get the child
 * @param tag identifier of the child node to get
 * @return Node child node with the specified tag
 */
Node get_child_node(Node parentNode, std::string tag);

/**
 * @brief adds the landmarks in the list to the track.
 *
 * @param track vector to which the landmarks will be added [x1, y1, x2, y2, ...]
 * @param list node with landmarks as children each with a "position" key
 */
void add_landmarks(Eigen::VectorXd& track, const Node& list);

/**
 * @brief loads the map from the specified path and fills the start pose and track.
 *
 * @param mapPath path to the map file
 * @param start_pose vector to fill with the start pose [x, y, theta]
 * @param track vector to fill with the track data [x1, y1, x2, y2, ...]
 */
void load_map(std::string mapPath, Eigen::Vector3d& start_pose, Eigen::VectorXd& track);

/**
 * @brief loads the acceleration track from the default path.
 *
 * @param start_pose vector to fill with the start pose [x, y, theta]
 * @param track vector to fill with the track data [x1, y1, x2, y2, ...]
 */
void load_acceleration_track(Eigen::Vector3d& start_pose, Eigen::VectorXd& track);

/**
 * @brief loads the skidpad track from the default path.
 *
 * @param start_pose vector to fill with the start pose [x, y, theta]
 * @param track vector to fill with the track data [x1, y1, x2, y2, ...]
 */
void load_skidpad_track(Eigen::Vector3d& start_pose, Eigen::VectorXd& track);

/**
 * @brief Transforms the track as seen from the start pose.
 *
 * @param track will be transformed in place to the start pose
 * @param start_pose will be set to zero after the transformation
 */
void transform_track(Eigen::VectorXd& track, Eigen::Vector3d& start_pose);

#endif /* TRACKLOADER_HPP */
