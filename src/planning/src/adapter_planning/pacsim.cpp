#include "adapter_planning/pacsim.hpp"

PacSimAdapter::PacSimAdapter(Planning* planning) : Adapter(planning) {
  if (this->node->using_simulated_se_) {
    RCLCPP_DEBUG(this->node->get_logger(), "Using simulated SE in pacsim");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    auto timer = this->node->create_wall_timer(std::chrono::milliseconds(60),
                                               std::bind(&PacSimAdapter::timer_callback, this));

    this->finished_client_ =
        this->node->create_client<std_srvs::srv::Empty>("/pacsim/finish_signal");

    this->path_sub_ = this->node->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/pacsim/map", 10, std::bind(&PacSimAdapter::track_callback, this, std::placeholders::_1));
  }
  RCLCPP_DEBUG(this->node->get_logger(), "Reached pacsim adapter");
  this->node->mission = common_lib::competition_logic::Mission::AUTOCROSS;
}

void PacSimAdapter::timer_callback() {
  if (tf_buffer_->canTransform("map", "car", tf2::TimePointZero)) {
    custom_interfaces::msg::VehicleState pose;
    geometry_msgs::msg::TransformStamped t =
        tf_buffer_->lookupTransform("map", "car", tf2::TimePointZero);
    pose.header = t.header;
    tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
                      t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll;
    double pitch;
    double yaw;
    m.getRPY(roll, pitch, yaw);
    pose.theta = yaw;
    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.linear_velocity = 0.0;   // not needed -> default value
    pose.angular_velocity = 0.0;  // not needed -> default value
    this->node->vehicle_localization_callback(pose);
  }
}

void PacSimAdapter::set_mission_state(int mission, int state) {
  std::cout << "Set mission undefined for PacSim\n";
}

void PacSimAdapter::finish() {
  this->finished_client_->async_send_request(
      std::make_shared<std_srvs::srv::Empty::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
        RCLCPP_INFO(this->node->get_logger(), "Finished signal sent");
      });
}

void PacSimAdapter::track_callback(const visualization_msgs::msg::MarkerArray& msg) {
  RCLCPP_DEBUG(this->node->get_logger(), "Received track map in pacsim adapter");
  custom_interfaces::msg::ConeArray cones;
  for (auto c : msg.markers) {
    custom_interfaces::msg::Cone cone;
    cone.position.x = c.pose.position.x;
    cone.position.y = c.pose.position.y;
    cones.cone_array.push_back(cone);
  }
  std::cout << std::endl;
  this->node->track_map_callback(cones);
}