#pragma once

#include <libwebsockets.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

class SimpleWebSocketServer : public rclcpp::Node {
public:
  SimpleWebSocketServer();

private:
  struct lws_context* context_;
  std::string config_json_;
  int port_ = 8767;

  void load_config();
  void start_server();
  void send_config_to_client(struct lws* wsi);

  static int websocket_callback(struct lws* wsi, enum lws_callback_reasons reason, void* user,
                                void* in, size_t len);
};