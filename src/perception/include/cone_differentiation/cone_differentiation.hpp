#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Enumeration for cone colors.
 */
typedef enum {
    BLUE,      ///< Blue cone.
    YELLOW,    ///< Yellow cone.
    UNDEFINED  ///< Undefined color.
} Color;

/**
 * @class ConeDifferentiation
 * 
 * @brief Abstract class for cone differentiation based on color.
 */
class ConeDifferentiation {
 public:
    /**
     * @brief Perform cone differentiation on a cone's point cloud.
     * 
     * @param cone_point_cloud Cluster's point cloud.
     * @return Color The cone's color.
     */
    virtual Color coneDifferentiation(const pcl::PointCloud<pcl::PointXYZI>::Ptr
                                       cone_point_cloud) const = 0;
};
