#include "planning/skidpad.hpp"

#include <fstream>
#include <sstream>
#include <limits>
#include <cmath>

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "config/path_calculation_config.hpp"

Eigen::Matrix4f Skidpad::align_cones_with_icp(const std::vector<Cone>& cone_array) {
  // Check if we have enough cones for ICP
  if (reference_cones_.size() < static_cast<std::size_t>(config_.minimum_cones_) ||
      cone_array.size() < static_cast<std::size_t>(config_.minimum_cones_)) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Not enough cones to perform ICP alignment.");
    return Eigen::Matrix4f::Identity();
  }

  // Convert cone_array (LiDAR) to source cloud
  pcl::PointCloud<pcl::PointXYZ> cloud_source;
  cloud_source.reserve(cone_array.size());
  for (const auto& cone : cone_array) {
    (void)cloud_source.emplace_back(cone.position.x, cone.position.y, 0.0);
  }

  // Convert reference cones to target cloud
  pcl::PointCloud<pcl::PointXYZ> cloud_target;
  cloud_target.reserve(reference_cones_.size());
  for (const auto& [x, y] : reference_cones_) {
    (void)cloud_target.emplace_back(x, y, 0.0);
  }

  // Run ICP alignment
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_source.makeShared());
  icp.setInputTarget(cloud_target.makeShared());
  icp.setMaxCorrespondenceDistance(config_.tolerance_);
  icp.setMaximumIterations(std::numeric_limits<int>::max());
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-3);

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  if (!icp.hasConverged()) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "ICP did not converge.");
    return Eigen::Matrix4f::Identity();
  }

  return icp.getFinalTransformation();
}

size_t Skidpad::find_closest_path_index(
    const std::vector<PathPoint>& path,
    const common_lib::structures::Pose& pose) {
  
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_index = 0;
  
  for (size_t i = 0; i < path.size(); ++i) {
    double dx = path[i].position.x - pose.position.x;
    double dy = path[i].position.y - pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist > min_dist) {
      break;  // Stop searching if distance starts increasing
    }
    
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = i;
    } 
  }
  
  return closest_index;
}

std::vector<PathPoint> Skidpad::skidpad_path(const std::vector<Cone>& cone_array,
                                                     common_lib::structures::Pose pose) {
  std::vector<PathPoint> result;

  // Load data from files only on first iteration
  if (!skidpad_data_loaded_) {
    const std::string file = "./src/planning/src/utils/skidpad.txt";
    const std::string conesfile = "./src/planning/src/utils/skidpadcones1.txt";

    // 1. Read reference cones from file
    std::ifstream cfile(conesfile);
    reference_cones_.clear();
    std::string line;
    while (std::getline(cfile, line)) {
      std::istringstream iss(line);
      double x = 0.0, y = 0.0, z = 0.0;
      if (iss >> x >> y >> z) {
        (void)reference_cones_.emplace_back(x, y);
      } else {
        break;
      }
    }

    // 2. Read hardcoded path from file
    hardcoded_path_.clear();
    std::ifstream infile(file);
    while (std::getline(infile, line)) {
      std::istringstream iss(line);
      double x = 0.0, y = 0.0, v = 0.0;
      if (iss >> x >> y >> v) {
        (void)hardcoded_path_.emplace_back(x, y, v);
      } else {
        break;
      }
    }

    skidpad_data_loaded_ = true;
  }

  // Perform ICP alignment
  Eigen::Matrix4f icp_transform = align_cones_with_icp(cone_array);
  
  if (icp_transform.isIdentity()) {
    return result;  // empty to indicate failure
  }

  // Apply transformation to hardcoded path (create a copy)
  result = hardcoded_path_;
  Eigen::Matrix4f map_to_lidar_transform = icp_transform.inverse();

  for (auto& point : result) {
    Eigen::Vector4f p(point.position.x, point.position.y, 0.0f, 1.0f);
    Eigen::Vector4f transformed = map_to_lidar_transform * p;
    point.position.x = transformed[0];
    point.position.y = transformed[1];
  }

  // Update predefined_path_ with the transformed path
  predefined_path_ = result;

  // Get the closest points to the car pose
  if (predefined_path_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Predefined path is empty.");
    return result;
  }

  size_t closest_index = find_closest_path_index(predefined_path_, pose);

  RCLCPP_INFO(rclcpp::get_logger("planning"),
              "predefined path size is %zu, closest index is %zu \n Hardcoded path size is %zu",
              predefined_path_.size(), closest_index, hardcoded_path_.size());
  
  // Remove all the points before the closest point not removing the closest point itself
  (void)predefined_path_.erase(predefined_path_.begin(), predefined_path_.begin() + closest_index);
  (void)hardcoded_path_.erase(hardcoded_path_.begin(), hardcoded_path_.begin() + closest_index);

  size_t path_size = predefined_path_.size();
  size_t count = 70;
  if (path_size < 70) {
    count = path_size;
  }

  return std::vector<PathPoint>(predefined_path_.begin(), predefined_path_.begin() + count);
}