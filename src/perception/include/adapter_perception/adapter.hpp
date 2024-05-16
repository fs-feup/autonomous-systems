#ifndef SRC_PERCEPTION_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_PERCEPTION_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "custom_interfaces/msg/vcu.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "custom_interfaces/msg/operational_status.hpp"

class Perception;

/**
 * @brief Adapter class for coordinating communication between different modes
 * and the Perception module.
 * 
 * This class serves as an intermediary for communication between different modes,
 * such as simulators or lidar itself.
 */
class Adapter {
 protected:
  ///< PointCloud2 subscription.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _point_cloud_subscription;

 public:
   /**
   * @brief Constructor for Adapter class.
   * 
   * @param perception A pointer to the Perception instance.
   */
  explicit Adapter(Perception* perception);

  Perception* node; // Pointer to the Perception instance.

    /**
     * @brief Finalizes the Adapter.
     * 
     * This method is pure virtual and must be implemented by derived classes.
     */
    virtual void finish() = 0;
};

#endif  // SRC_PERCEPTION_INCLUDE_ADAPTER_ADAPTER_HPP_
