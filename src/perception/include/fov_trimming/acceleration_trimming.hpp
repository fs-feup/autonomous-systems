#pragma once

#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <utils/trimming_parameters.hpp>

#include "fov_trimming/fov_trimming.hpp"

class AccelerationTrimming : public FovTrimming {
public:
  /**
   * @brief Constructor for the Acceleration Point Cloud Trimming algorithm.
   * @param params All trimming related parameters.
   */
  AccelerationTrimming(const TrimmingParameters params);

  /**
   * @brief Default constructor.
   *
   * This constructor is provided as a default constructor.
   */
  AccelerationTrimming() = default;

  /**
   * @brief Perform ground removal on the input point cloud.
   *
   * This pure virtual function must be implemented by derived classes.
   *
   * @param point_cloud The input point cloud to be processed (trimmed).
   * @param[out] ret The resulting point cloud after trimming and the corresponding split parameters
   * for GridRANSAC.
   */
  PclSplitParameters fov_trimming(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud) const override;
};