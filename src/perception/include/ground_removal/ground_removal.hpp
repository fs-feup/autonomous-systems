#include "sensor_msgs/msg/point_cloud2.hpp"

class GroundRemoval {

    public:
        virtual sensor_msgs::msg::PointCloud2 groundRemoval(sensor_msgs::msg::PointCloud2 point_cloud) const = 0;
};