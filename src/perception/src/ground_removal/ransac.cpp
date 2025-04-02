#include "ground_removal/ransac.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

RANSAC::RANSAC(const double epsilon, const int n_tries) : epsilon(epsilon), n_tries(n_tries) {}

void RANSAC::ground_removal(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr ret, Plane& plane,
                            [[maybe_unused]] const SplitParameters split_params) const {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_indices(new pcl::PointIndices);

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
}