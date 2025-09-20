#include <libwebsockets.h>

#include <fstream>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

class SimpleWebSocketServer : public rclcpp::Node {
public:
  SimpleWebSocketServer() : Node("simple_websocket_server") {
    RCLCPP_INFO(this->get_logger(), "Starting simple WebSocket server...");
    load_config();
    start_server();
  }

private:
  struct lws_context* context_;
  std::string config_json_;
  int port_ = 8768;

  void load_config() {
    std::ifstream config_file("node.json");
    if (config_file.is_open()) {
      nlohmann::json config;
      config_file >> config;
      config_json_ = config.dump();
      config_file.close();
      RCLCPP_INFO(this->get_logger(), "Loaded config: %s", config_json_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not open node.json");
    }
  }

  static int websocket_callback(struct lws* wsi, enum lws_callback_reasons reason, void* user,
                                void* in, size_t len) {
    SimpleWebSocketServer* server =
        static_cast<SimpleWebSocketServer*>(lws_context_user(lws_get_context(wsi)));

    switch (reason) {
      case LWS_CALLBACK_ESTABLISHED:
        RCLCPP_INFO(server->get_logger(), "Client connected");
        server->send_config_to_client(wsi);
        break;

      case LWS_CALLBACK_CLOSED:
        RCLCPP_INFO(server->get_logger(), "Client disconnected");
        break;

      case LWS_CALLBACK_RECEIVE:
        RCLCPP_INFO(server->get_logger(), "Received: %.*s", (int)len, (char*)in);
        break;

      default:
        break;
    }
    return 0;
  }

  void send_config_to_client(struct lws* wsi) {
    if (config_json_.empty()) return;

    size_t message_size = config_json_.size();
    size_t padded_size = LWS_PRE + message_size;
    std::vector<uint8_t> padded_message(padded_size);

    std::memcpy(padded_message.data() + LWS_PRE, config_json_.c_str(), message_size);

    int written = lws_write(wsi, padded_message.data() + LWS_PRE, message_size, LWS_WRITE_TEXT);
    if (written < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send config to client");
    } else {
      RCLCPP_INFO(this->get_logger(), "Sent config to client");
    }
  }

  void start_server() {
    struct lws_context_creation_info info;
    memset(&info, 0, sizeof(info));

    static struct lws_protocols protocols[] = {
        {"simple-protocol", websocket_callback, 0, 1024, 0, NULL, 0},
        {NULL, NULL, 0, 0, 0, NULL, 0}};

    info.port = port_;
    info.protocols = protocols;
    info.user = this;

    context_ = lws_create_context(&info);
    if (!context_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create WebSocket context");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "WebSocket server started on port %d", port_);

    std::thread([this]() {
      while (rclcpp::ok()) {
        lws_service(context_, 50);
      }
    }).detach();
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleWebSocketServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}