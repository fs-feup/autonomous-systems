#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/PCLPointField.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class GroundRemoval {

    public:
        virtual void groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, pcl::PointCloud<pcl::PointXYZI> &ret) const = 0;
};