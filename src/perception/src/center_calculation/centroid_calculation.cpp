#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/impl/centroid.hpp>
#include <string>

#include <center_calculation/centroid_calculation.hpp>


Eigen::Vector4f CentroidCalculator::calculate_center(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, [[maybe_unused]] const Plane& plane) const {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*point_cloud, centroid);
    return centroid;
}