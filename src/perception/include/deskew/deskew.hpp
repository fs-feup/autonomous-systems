#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "common_lib/structures/velocities.hpp"
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "utils/cluster.hpp"

class Deskew {
public:
    /**
     * @brief deskews the point cloud, i.e. removes error coming from LiDAR rotation while
     * the vehicle moves by moving all points to where they'd be seen at the time of the end
     * of the scan. Assumes constant velocity.
     * 
     * @param input_cloud Cloud to be deskewed
     * @param vehicle_velocity Vehicle's velocity in its body reference frame
     */
    void deskew_point_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
         const common_lib::structures::Velocities& vehicle_velocity);
};