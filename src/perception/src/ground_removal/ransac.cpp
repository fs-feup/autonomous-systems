#include "ground_removal/ransac.hpp"
#include <pcl/common/io.h>
#include <pcl/conversions.h>

RANSAC::RANSAC(double epsilon, int n_tries) : epsilon(epsilon), n_tries(n_tries) {}


sensor_msgs::msg::PointCloud2 RANSAC::groundRemoval(sensor_msgs::msg::PointCloud2 point_cloud) const{


    
    return point_cloud;
}