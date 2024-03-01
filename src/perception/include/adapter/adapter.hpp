#ifndef SRC_PERCEPTION_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_PERCEPTION_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "custom_interfaces/msg/vcu.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"

#include "rclcpp/rclcpp.hpp"

class Perception;

/**
 * @brief Adapter class for coordinating communication between different modes
 * and Planning module.
 */
class Adapter {
 protected:
    ///< PointCloud2 subscription.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _point_cloud_subscription;
 public:
    explicit Adapter(Perception* perception);
    Perception* node;

    virtual void init() = 0;
    virtual void finish() = 0;
};

#endif  // SRC_PERCEPTION_INCLUDE_ADAPTER_ADAPTER_HPP_
