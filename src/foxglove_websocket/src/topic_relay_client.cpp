#include "foxglove_websocket/topic_relay_client.hpp"

#include <libwebsockets.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>

TopicRelayClient::TopicRelayClient() : Node("topic_relay_client") {
  RCLCPP_INFO(this->get_logger(), "Starting Topic Relay Client...");
  connect_websocket();
}

TopicRelayClient::~TopicRelayClient() {
  stop_foxglove_bridge();
  if (context_) {
    lws_context_destroy(context_);
  }
}

void TopicRelayClient::connect_websocket() {
  struct lws_context_creation_info info;
  memset(&info, 0, sizeof(info));

  static struct lws_protocols protocols[] = {
      {"simple-protocol", TopicRelayClient::websocket_callback, 0, 1024, 0, NULL, 0},
      {NULL, NULL, 0, 0, 0, NULL, 0}  // Terminator
  };

  info.port = CONTEXT_PORT_NO_LISTEN;
  info.protocols = protocols;
  info.user = this;

  context_ = lws_create_context(&info);
  if (!context_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create WebSocket context");
    return;
  }

  // Connect to server
  struct lws_client_connect_info ccinfo;
  memset(&ccinfo, 0, sizeof(ccinfo));
  ccinfo.context = context_;
  ccinfo.address = server_uri_.c_str();
  ccinfo.port = server_port_;
  ccinfo.path = "/";
  ccinfo.host = server_uri_.c_str();
  ccinfo.origin = server_uri_.c_str();
  ccinfo.protocol = "simple-protocol";

  websocket_ = lws_client_connect_via_info(&ccinfo);
  if (!websocket_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to WebSocket server");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Connecting to WebSocket server at %s:%d", server_uri_.c_str(),
              server_port_);

  // Run WebSocket service in separate thread
  std::thread([this]() {
    while (rclcpp::ok()) {
      lws_service(context_, 50);
    }
  }).detach();
}

int TopicRelayClient::websocket_callback(struct lws* wsi, enum lws_callback_reasons reason,
                                         void* user, void* in, size_t len) {
  TopicRelayClient* client = static_cast<TopicRelayClient*>(lws_context_user(lws_get_context(wsi)));

  switch (reason) {
    case LWS_CALLBACK_CLIENT_ESTABLISHED:
      RCLCPP_INFO(client->get_logger(), "Connected to WebSocket server");
      break;

    case LWS_CALLBACK_CLIENT_RECEIVE: {
      std::string config_data(static_cast<char*>(in), len);
      RCLCPP_INFO(client->get_logger(), "Received config: %s", config_data.c_str());
      client->process_received_config(config_data);
    } break;

    case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
      RCLCPP_ERROR(client->get_logger(), "WebSocket connection error");
      break;

    case LWS_CALLBACK_CLOSED:
      RCLCPP_INFO(client->get_logger(), "WebSocket connection closed");
      break;

    default:
      break;
  }
  return 0;
}

void TopicRelayClient::process_received_config(const std::string& config_json) {
  try {
    topics_config_ = nlohmann::json::parse(config_json);

    RCLCPP_INFO(this->get_logger(), "Processing topic configuration...");

    // Setup topic relays for enabled topics
    for (const auto& [topic_name, config] : topics_config_.items()) {
      if (config.is_object() && config.value("enabled", false)) {
        std::string message_type = config.value("type", "string");
        RCLCPP_INFO(this->get_logger(), "Setting up relay for %s (type: %s)", topic_name.c_str(),
                    message_type.c_str());
        setup_topic_relay(topic_name, message_type);
      }
    }

    // Start Foxglove bridge after setting up relays
    start_foxglove_bridge();

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse config JSON: %s", e.what());
  }
}

void TopicRelayClient::setup_topic_relay(const std::string& topic_name,
                                         const std::string& message_type) {
  std::string relay_topic = "/relay" + topic_name;

  if (message_type == "vector3_stamped") {
    // Create publisher for relayed topic
    auto publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(relay_topic, 10);
    publishers_[topic_name] = publisher;

    // Create subscriber to original topic
    auto subscriber = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        topic_name, 10,
        [this, topic_name](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
          this->vector3_stamped_callback(msg, topic_name);
        });
    subscribers_[topic_name] = subscriber;

  } else if (message_type == "string") {
    // Create publisher for relayed topic
    auto publisher = this->create_publisher<std_msgs::msg::String>(relay_topic, 10);
    publishers_[topic_name] = publisher;

    // Create subscriber to original topic
    auto subscriber = this->create_subscription<std_msgs::msg::String>(
        topic_name, 10, [this, topic_name](const std_msgs::msg::String::SharedPtr msg) {
          this->string_callback(msg, topic_name);
        });
    subscribers_[topic_name] = subscriber;

  } else if (message_type == "velocities") {
    // For now, treat as String until custom_interfaces is available
    auto publisher = this->create_publisher<std_msgs::msg::String>(relay_topic, 10);
    publishers_[topic_name] = publisher;

    auto subscriber = this->create_subscription<std_msgs::msg::String>(
        topic_name, 10, [this, topic_name](const std_msgs::msg::String::SharedPtr msg) {
          this->velocities_callback(msg, topic_name);
        });
    subscribers_[topic_name] = subscriber;
  }

  RCLCPP_INFO(this->get_logger(), "Setup relay: %s -> %s", topic_name.c_str(), relay_topic.c_str());
}

void TopicRelayClient::vector3_stamped_callback(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr msg, const std::string& topic_name) {
  // Republish on relay topic
  std::string relay_topic = "/relay" + topic_name;
  auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>>(
      publishers_[topic_name]);
  if (publisher) {
    publisher->publish(*msg);
  }
}

void TopicRelayClient::string_callback(const std_msgs::msg::String::SharedPtr msg,
                                       const std::string& topic_name) {
  // Republish on relay topic
  std::string relay_topic = "/relay" + topic_name;
  auto publisher =
      std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(publishers_[topic_name]);
  if (publisher) {
    publisher->publish(*msg);
  }
}

void TopicRelayClient::velocities_callback(const std_msgs::msg::String::SharedPtr msg,
                                           const std::string& topic_name) {
  // Republish on relay topic (treating as string for now)
  std::string relay_topic = "/relay" + topic_name;
  auto publisher =
      std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(publishers_[topic_name]);
  if (publisher) {
    publisher->publish(*msg);
  }
}

void TopicRelayClient::start_foxglove_bridge() {
  RCLCPP_INFO(this->get_logger(), "Starting Foxglove bridge...");

  foxglove_bridge_pid_ = fork();
  if (foxglove_bridge_pid_ == 0) {
    // Child process - launch Foxglove bridge
    execlp("ros2", "ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml", nullptr);
    exit(1);  // If execlp fails
  } else if (foxglove_bridge_pid_ > 0) {
    RCLCPP_INFO(this->get_logger(), "Foxglove bridge started with PID %d", foxglove_bridge_pid_);
    RCLCPP_INFO(this->get_logger(), "Connect Foxglove Studio to ws://localhost:8765");
    RCLCPP_INFO(this->get_logger(), "Relay topics will appear with /relay prefix");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to start Foxglove bridge");
  }
}

void TopicRelayClient::stop_foxglove_bridge() {
  if (foxglove_bridge_pid_ > 0) {
    RCLCPP_INFO(this->get_logger(), "Stopping Foxglove bridge...");
    kill(foxglove_bridge_pid_, SIGTERM);
    int status;
    waitpid(foxglove_bridge_pid_, &status, 0);
    foxglove_bridge_pid_ = -1;
    RCLCPP_INFO(this->get_logger(), "Foxglove bridge stopped");
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopicRelayClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}