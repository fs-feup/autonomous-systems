#ifndef TRACKLOADER_HPP
#define TRACKLOADER_HPP

#include <Eigen/Dense>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

using namespace YAML;

Node get_child_node(Node parentNode, std::string tag);

void add_landmarks(Eigen::VectorXd& track, Node* list, int* _coneCounter);

void load_map(std::string mapPath, Eigen::Vector3d& start_pose, Eigen::VectorXd& track);

void load_acceleration_track(Eigen::Vector3d& start_pose, Eigen::VectorXd& track);

void load_skidpad_track(Eigen::Vector3d& start_pose, Eigen::VectorXd& track);

#endif /* TRACKLOADER_HPP */
