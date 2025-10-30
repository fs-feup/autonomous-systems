#pragma once

#include "fov_trimming/fov_trimming.hpp"

class CutTrimming : public FovTrimming {
public:
  /**
   * @brief Constructor for the Cut Point Cloud Trimming algorithm.
   * @param params All trimming related parameters.
   */
  CutTrimming(const TrimmingParameters params);

  /**
   * @brief Default constructor.
   *
   * This constructor is provided as a default constructor.
   */
  CutTrimming() = default;

  /**
   * @brief Perform ground removal on the input point cloud.
   *
   * This pure virtual function must be implemented by derived classes.
   *
   * @param point_cloud The input point cloud to be processed (trimmed).
   * @param[out] ret The resulting point cloud after trimming and the corresponding split parameters
   * for GridRANSAC.
   */
  SplitParameters fov_trimming(
      const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
      sensor_msgs::msg::PointCloud2::SharedPtr& trimmed_cloud) const override;
};