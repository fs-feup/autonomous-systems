#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef enum {
    BLUE,
    YELLOW,
    UNDEFINED
} Color;


class ConeDifferentiation {
 public:
    virtual Color coneDifferentiation(const pcl::PointCloud<pcl::PointXYZI>::Ptr
                                       cone_point_cloud) const = 0;
};