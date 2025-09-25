#pragma once

#include <iostream>
#include <utils/pcl_point_types.hpp>

/**
 * @class ICP
 * @brief Class for performing Iterative Closest Point (ICP) registration.
 */
class ICP {
private:
  pcl::IterativeClosestPoint<PointXYZIR, PointXYZIR> _icp_; /**< ICP object */

public:
  /**
   * @brief Constructor for the ICP class.
   * @param target_file Path to the target point cloud file.
   * @param max_correspondence_distance Maximum correspondence distance for ICP.
   * @param max_iteration Maximum number of iterations for ICP.
   * @param transformation_epsilon Transformation epsilon for convergence criteria.
   * @param euclidean_fitness_epsilon Euclidean fitness epsilon for convergence criteria.
   */
  ICP(std::string target_file, double max_correspondence_distance, long max_iteration,
      double transformation_epsilon, double euclidean_fitness_epsilon);

  /**
   * @brief Execute the ICP registration.
   * @param source Source point cloud for registration.
   * @param final_cloud Final registered point cloud.
   * @return Fitness score after registration.
   */
  double executeICP(pcl::PointCloud<PointXYZIR>::Ptr source,
                    pcl::PointCloud<PointXYZIR>::Ptr final_cloud);
};
