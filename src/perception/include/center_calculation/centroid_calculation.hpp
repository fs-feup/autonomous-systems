#ifndef CENTROID_CALCULATOR_HPP
#define CENTROID_CALCULATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/impl/centroid.hpp>
#include <string>

#include <center_calculation/center_calculation.hpp>

class CentroidCalculator : public CenterCalculator {
 public:
  void calculateCenter(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud) override;
};

#endif  // CENTROID_CALCULATOR_HPP