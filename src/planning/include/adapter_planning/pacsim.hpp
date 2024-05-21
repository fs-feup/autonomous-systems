#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_PACSIM_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_PACSIM_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "adapter_planning/adapter.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
class PacSimAdapter : public Adapter {
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr finished_client_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr path_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

public:
  explicit PacSimAdapter(std::shared_ptr<Planning> planning);

  void set_mission_state(int mission, int state);
  void track_callback(const visualization_msgs::msg::MarkerArray& msg);
  void finish() override;
  void timer_callback();
};

#endif