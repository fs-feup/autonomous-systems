#ifndef CENTER_CALCULATOR_HPP
#define CENTER_CALCULATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/impl/centroid.hpp>
#include <string>

class CenterCalculator {
 public:
  virtual void calculateCenter(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud) = 0;
};

#endif  // CENTER_CALCULATOR_HPP
