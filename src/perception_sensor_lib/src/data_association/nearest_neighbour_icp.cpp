#include "perception_sensor_lib/data_association/nearest_neighbour_icp.hpp"

Eigen::VectorXd NearestNeighbourICP::transform_points(const Eigen::VectorXd& landmarks,
                                                      const Eigen::VectorXd& observations) const {
  const int num_landmarks = landmarks.size() / 2;
  const int num_observations = observations.size() / 2;

  // Convert landmarks (reference) to target cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>());
  cloud_target->reserve(num_landmarks);
  for (int i = 0; i < num_landmarks; ++i) {
    cloud_target->emplace_back(landmarks(2 * i), landmarks(2 * i + 1), 0.0);
  }

  // Convert observations (to be transformed) to source cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>());
  cloud_source->reserve(num_observations);
  for (int i = 0; i < num_observations; ++i) {
    cloud_source->emplace_back(observations(2 * i), observations(2 * i + 1), 0.0);
  }

  // Run ICP alignment
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_source);
  icp.setInputTarget(cloud_target);
  icp.setMaxCorrespondenceDistance(10);
  icp.setMaximumIterations(100);  // reasonable upper limit
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-3);

  pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
  icp.align(aligned_cloud);

  // Check if ICP has converged
  if (!icp.hasConverged()) {
    RCLCPP_WARN(rclcpp::get_logger("NearestNeighbourICP"), "ICP did not converge");
    return observations;  // Return original observations if ICP fails
  }

  // Get the final transformation
  Eigen::Matrix4f T = icp.getFinalTransformation();

  // Apply T to the observations
  Eigen::VectorXd transformed_observations(2 * num_observations);
  for (int i = 0; i < num_observations; ++i) {
    Eigen::Vector4f p;
    p << observations(2 * i), observations(2 * i + 1), 0.0f, 1.0f;

    Eigen::Vector4f p_transformed = T * p;

    transformed_observations(2 * i) = p_transformed(0);
    transformed_observations(2 * i + 1) = p_transformed(1);
  }

  return transformed_observations;
}

Eigen::VectorXi NearestNeighbourICP::associate(
    const Eigen::VectorXd& landmarks, const Eigen::VectorXd& observations,
    const Eigen::MatrixXd& covariance, const Eigen::VectorXd& observation_confidences) const {
  const int num_observations = observations.size() / 2;
  const int num_landmarks = landmarks.size() / 2;

  Eigen::VectorXi associations = Eigen::VectorXi::Constant(observations.size() / 2, -2);
  if (num_observations == 0) {
    return associations;
  }
  Eigen::VectorXd transformed_observations;

  if (num_landmarks > 8) {
    transformed_observations = this->transform_points(landmarks, observations);
  } else {
    transformed_observations = observations;  // No transformation for small number of landmarks
  }

  for (int i = 0; i < num_observations; i++) {
    double min_distance = std::numeric_limits<double>::max();
    int min_index = -1;
    for (int j = 0; j < num_landmarks; j++) {
      const double euclidean_distance =
          std::hypot(transformed_observations(2 * i) - landmarks(2 * j),
                     transformed_observations(2 * i + 1) - landmarks(2 * j + 1));
      if (euclidean_distance < min_distance) {
        min_distance = euclidean_distance;
        min_index = 2 * j;
      }
    }

    if (observation_confidences(i) >= this->_params_.new_landmark_confidence_gate) {
      if (min_distance < this->_params_.association_gate) {
        associations(i) = min_index;
      } else {
        associations(i) = -1;
      }
    }
  }
  return associations;
}