#include "ground_removal/ransac.hpp"

RANSAC::RANSAC(const double epsilon, const int n_tries) : epsilon(epsilon), n_tries(n_tries) {}

void RANSAC::ground_removal(const sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_point_cloud,
                            sensor_msgs::msg::PointCloud2::SharedPtr& ground_removed_cloud,
                            Plane& plane) const {
  /**
pcl::ModelCoefficients::Ptr coefficients = std::make_shared<pcl::ModelCoefficients>();
pcl::PointIndices::Ptr inliers_indices = std::make_shared<pcl::PointIndices>();

// Segmentation Object creation
pcl::SACSegmentation<pcl::PointXYZI> seg;

// Optional: Increases Accuracy
seg.setOptimizeCoefficients(true);

// Defining RANSAC properties in the segmentation Object
seg.setModelType(pcl::SACMODEL_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setDistanceThreshold(epsilon);
seg.setMaxIterations(n_tries);

seg.setInputCloud(point_cloud);
seg.segment(*inliers_indices, *coefficients);

if (coefficients->values.size() <= 0) {
plane = Plane(0, 0, 0, 0);
}

else {
plane = Plane(coefficients->values[0], coefficients->values[1], coefficients->values[2],
coefficients->values[3]);
}

pcl::ExtractIndices<pcl::PointXYZI> extract;
extract.setInputCloud(point_cloud);
extract.setIndices(inliers_indices);

extract.setNegative(true);  // Extract outliers
extract.filter(*ret);
*/
}