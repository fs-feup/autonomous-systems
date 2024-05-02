#ifndef CIRCUNFERENCE_CENTER_CALCULATION_HPP
#define CIRCUNFERENCE_CENTER_CALCULATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/impl/centroid.hpp>
#include <string>
#include <center_calculation/center_calculation.hpp>

class CircunferenceCenterCalculation : public CenterCalculator {
 public:
  void calculateCenter(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud) override;
};

#endif  // CIRCUNFERENCE_CENTER_CALCULATION_HPP