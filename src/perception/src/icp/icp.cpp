#include "icp/icp.hpp"

ICP::ICP(std::string target_file, double max_correspondence_distance, long max_iteration,
         double transformation_epsilon, double euclidean_fitness_epsilon) {
  pcl::PointCloud<PointXYZIR>::Ptr target_cloud(new pcl::PointCloud<PointXYZIR>);

  if (pcl::io::loadPCDFile<PointXYZIR>(target_file, *target_cloud) < 0) {
    PCL_ERROR("Couldn't read file\n");
  }

  _icp_.setInputTarget(target_cloud);

  _icp_.setMaxCorrespondenceDistance(max_correspondence_distance);

  _icp_.setMaximumIterations(max_iteration);

  _icp_.setTransformationEpsilon(transformation_epsilon);

  _icp_.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
}

double ICP::executeICP(pcl::PointCloud<PointXYZIR>::Ptr source_cloud,
                       pcl::PointCloud<PointXYZIR>::Ptr final_cloud) {
  _icp_.setInputSource(source_cloud);

  // Align the clouds
  pcl::PointCloud<PointXYZIR>::Ptr aligned_cloud(new pcl::PointCloud<PointXYZIR>);
  _icp_.align(*aligned_cloud);

  if (_icp_.hasConverged()) {
    *final_cloud = *aligned_cloud;
    return _icp_.getFitnessScore();
  } else {
    return -1;
  }
}