#pragma once

#include <libwebsockets.h>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

// Forward declarations for custom interfaces
namespace custom_interfaces {
namespace msg {
struct Velocities {
  double linear_x;
  double linear_y;
  double angular_z;
};
}  // namespace msg
}  // namespace custom_interfaces

class TopicRelayClient : public rclcpp::Node {
public:
  TopicRelayClient();
  ~TopicRelayClient();

private:
  // WebSocket connection
  struct lws_context* context_;
  struct lws* websocket_;
  std::string server_uri_ = "localhost";
  int server_port_ = 8768;

  // Topic configuration
  nlohmann::json topics_config_;

  // Publishers map
  std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;

  // Subscribers map
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscribers_;

  // Foxglove bridge process
  int foxglove_bridge_pid_ = -1;

  // Methods
  void connect_websocket();
  void process_received_config(const std::string& config_json);
  void setup_topic_relay(const std::string& topic_name, const std::string& message_type);
  void start_foxglove_bridge();
  void stop_foxglove_bridge();

  // WebSocket callbacks
  static int websocket_callback(struct lws* wsi, enum lws_callback_reasons reason, void* user,
                                void* in, size_t len);

  // Topic callbacks
  void vector3_stamped_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg,
                                const std::string& topic_name);
  void string_callback(const std_msgs::msg::String::SharedPtr msg, const std::string& topic_name);
  void velocities_callback(const std_msgs::msg::String::SharedPtr msg,
                           const std::string& topic_name);
};