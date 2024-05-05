#ifndef CIRCUNFERENCE_CENTER_CALCULATION_HPP
#define CIRCUNFERENCE_CENTER_CALCULATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/impl/centroid.hpp>
#include <string>
#include <center_calculation/center_calculation.hpp>

/**
 * @brief Concrete class representing a circumference-based method for estimating the center position of a cone.
 * 
 * This class implements the CenterCalculator interface to provide a method for estimating the center position 
 * of a cone based on its point cloud data, using a circumference-based approach.
 */
class CircunferenceCenterCalculation : public CenterCalculator {
 public:
 /**
  * @brief Estimates the center position of a cone using a circumference-based approach.
  * 
  * This method estimates the center position of a cone by approximating its center as the center
  * of a circumference that cuts the cone.
  * 
  * @param point_cloud Pointer to the point cloud representing the cone's cluster.
  * @param plane Plane representing the ground. (Mandatory)
  * @return Eigen::Vector4f representing the estimated center of the cone.
  * 
  * @note The plane parameter is mandatory for this method.
  */
  Eigen::Vector4f calculateCenter(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, const Plane& plane) const override;
};

#endif  // CIRCUNFERENCE_CENTER_CALCULATION_HPP

