#include "remote_control/remote_control.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <future>
#include <memory>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>

#include "boost/asio.hpp"
#include "boost/beast/core.hpp"
#include "boost/beast/websocket.hpp"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"

namespace remote_control {
namespace {

constexpr double kSteeringMinRad = -0.335;
constexpr double kSteeringMaxRad = 0.335;
constexpr double kThrottleMin = -1.0;
constexpr double kThrottleMax = 1.0;

bool has_number(const rapidjson::Document& document, const char* field) {
  return document.HasMember(field) && document[field].IsNumber();
}

bool has_bool(const rapidjson::Document& document, const char* field) {
  return document.HasMember(field) && document[field].IsBool();
}

std::vector<std::string> local_ipv4_addresses() {
  std::vector<std::string> addresses;
  ifaddrs* interfaces = nullptr;
  if (getifaddrs(&interfaces) != 0) {
    return addresses;
  }

  for (const ifaddrs* interface = interfaces; interface != nullptr; interface = interface->ifa_next) {
    if (interface->ifa_addr == nullptr || interface->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    if ((interface->ifa_flags & IFF_LOOPBACK) != 0) {
      continue;
    }

    char address_buffer[INET_ADDRSTRLEN] = {};
    const auto* socket_address =
        reinterpret_cast<const sockaddr_in*>(interface->ifa_addr);
    if (inet_ntop(AF_INET, &socket_address->sin_addr, address_buffer, sizeof(address_buffer)) !=
        nullptr) {
      addresses.emplace_back(address_buffer);
    }
  }

  freeifaddrs(interfaces);
  return addresses;
}

}  // namespace

StopPid::StopPid(const StopPidConfig& config) : config_(config) {}

double StopPid::update(double error, double dt_seconds) {
  if (dt_seconds <= 0.0) {
    dt_seconds = 1e-3;
  }

  integral_ += error * dt_seconds;
  integral_ = clamp_control_value(integral_, config_.integral_min, config_.integral_max);

  double derivative = 0.0;
  if (has_previous_error_) {
    derivative = (error - previous_error_) / dt_seconds;
  }

  previous_error_ = error;
  has_previous_error_ = true;

  const double output = config_.kp * error + config_.ki * integral_ + config_.kd * derivative;
  return clamp_control_value(output, config_.output_min, config_.output_max);
}

void StopPid::reset() {
  integral_ = 0.0;
  previous_error_ = 0.0;
  has_previous_error_ = false;
}

double clamp_control_value(double value, double min_value, double max_value) {
  return std::clamp(value, min_value, max_value);
}

ParseResult parse_emergency_control_message(const std::string& message) {
  rapidjson::Document document;
  document.Parse(message.c_str());

  if (document.HasParseError()) {
    return {false, {}, rapidjson::GetParseError_En(document.GetParseError())};
  }
  if (!document.IsObject()) {
    return {false, {}, "payload must be a JSON object"};
  }
  if (!document.HasMember("type") || !document["type"].IsString() ||
      std::string(document["type"].GetString()) != "emergency_control") {
    return {false, {}, "type must be emergency_control"};
  }
  if (!has_number(document, "steering_rad")) {
    return {false, {}, "steering_rad must be numeric"};
  }
  if (!has_number(document, "throttle")) {
    return {false, {}, "throttle must be numeric"};
  }
  if (!has_bool(document, "e_ebs")) {
    return {false, {}, "e_ebs must be boolean"};
  }
  if (!has_bool(document, "take_control")) {
    return {false, {}, "take_control must be boolean"};
  }

  EmergencyControlPayload payload;
  payload.steering_rad =
      clamp_control_value(document["steering_rad"].GetDouble(), kSteeringMinRad, kSteeringMaxRad);
  payload.throttle = clamp_control_value(document["throttle"].GetDouble(), kThrottleMin, kThrottleMax);
  payload.e_ebs = document["e_ebs"].GetBool();
  payload.take_control = document["take_control"].GetBool();
  if (document.HasMember("sent_at_ms") && document["sent_at_ms"].IsInt64()) {
    payload.sent_at_ms = document["sent_at_ms"].GetInt64();
  }

  return {true, payload, ""};
}

custom_interfaces::msg::ControlCommand make_control_command(double steering_rad,
                                                            double rear_throttle) {
  custom_interfaces::msg::ControlCommand command;
  command.throttle_fl = 0.0;
  command.throttle_fr = 0.0;
  command.throttle_rl = clamp_control_value(rear_throttle, kThrottleMin, kThrottleMax);
  command.throttle_rr = clamp_control_value(rear_throttle, kThrottleMin, kThrottleMax);
  command.steering = clamp_control_value(steering_rad, kSteeringMinRad, kSteeringMaxRad);
  return command;
}

class WebsocketServer::Impl {
 public:
  Impl(std::string host, uint16_t port, int idle_timeout_ms, MessageCallback message_callback,
       EventCallback event_callback)
      : host_(std::move(host)),
        port_(port),
        idle_timeout_ms_(idle_timeout_ms),
        message_callback_(std::move(message_callback)),
        event_callback_(std::move(event_callback)) {}

  ~Impl() { stop(); }

  bool start() {
    if (running_.exchange(true)) {
      return true;
    }

    std::promise<bool> listening_promise;
    auto listening_future = listening_promise.get_future();
    server_thread_ = std::thread([this, promise = std::move(listening_promise)]() mutable {
      run(std::move(promise));
    });

    const bool listening = listening_future.get();
    if (!listening && server_thread_.joinable()) {
      server_thread_.join();
    }
    return listening;
  }

  void stop() {
    if (!running_.exchange(false)) {
      return;
    }

    boost::system::error_code ec;
    {
      std::lock_guard<std::mutex> lock(sessions_mutex_);
      if (acceptor_) {
        acceptor_->close(ec);
      }
      for (const auto& websocket : sessions_) {
        if (websocket) {
          websocket->next_layer().socket().close(ec);
        }
      }
    }
    io_context_.stop();

    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    for (auto& session_thread : session_threads_) {
      if (session_thread.thread.joinable()) {
        session_thread.thread.join();
      }
    }
    session_threads_.clear();
  }

 private:
  using Tcp = boost::asio::ip::tcp;
  using Websocket = boost::beast::websocket::stream<boost::beast::tcp_stream>;

  struct SessionThread {
    std::thread thread;
    std::shared_ptr<std::atomic_bool> finished;
  };

  void run(std::promise<bool> listening_promise) {
    bool listening_promise_set = false;
    try {
      const auto address = boost::asio::ip::make_address(host_);
      Tcp::endpoint endpoint(address, port_);
      acceptor_ = std::make_unique<Tcp::acceptor>(io_context_);
      acceptor_->open(endpoint.protocol());
      acceptor_->set_option(boost::asio::socket_base::reuse_address(true));
      acceptor_->bind(endpoint);
      acceptor_->listen(boost::asio::socket_base::max_listen_connections);
      event_callback_("websocket listening on " + host_ + ":" + std::to_string(port_));
      listening_promise.set_value(true);
      listening_promise_set = true;

      while (running_) {
        boost::system::error_code ec;
        Tcp::socket socket(io_context_);
        acceptor_->accept(socket, ec);
        if (ec) {
          if (running_) {
            event_callback_("websocket accept failed: " + ec.message());
          }
          continue;
        }
        start_session(std::move(socket));
      }
    } catch (const std::exception& exception) {
      if (running_) {
        event_callback_("websocket server stopped after error: " + std::string(exception.what()));
      }
      if (!listening_promise_set) {
        listening_promise.set_value(false);
        listening_promise_set = true;
      }
      running_ = false;
    }
  }

  void start_session(Tcp::socket socket) {
    auto websocket = std::make_shared<Websocket>(boost::beast::tcp_stream(std::move(socket)));
    {
      std::lock_guard<std::mutex> lock(sessions_mutex_);
      reap_finished_session_threads_locked();
      sessions_.push_back(websocket);
      auto finished = std::make_shared<std::atomic_bool>(false);
      session_threads_.push_back(SessionThread{
          std::thread([this, websocket, finished]() {
            handle_session(websocket);
            finished->store(true);
          }),
          finished});
    }
  }

  void reap_finished_session_threads_locked() {
    auto thread_it = session_threads_.begin();
    while (thread_it != session_threads_.end()) {
      if (!thread_it->finished->load()) {
        ++thread_it;
        continue;
      }
      if (thread_it->thread.joinable()) {
        thread_it->thread.join();
      }
      thread_it = session_threads_.erase(thread_it);
    }
  }

  void handle_session(const std::shared_ptr<Websocket>& websocket) {
    if (idle_timeout_ms_ > 0) {
      boost::beast::websocket::stream_base::timeout timeout;
      timeout.handshake_timeout = std::chrono::milliseconds(idle_timeout_ms_);
      timeout.idle_timeout = std::chrono::milliseconds(idle_timeout_ms_);
      timeout.keep_alive_pings = true;
      websocket->set_option(timeout);
    }

    try {
      websocket->accept();
      event_callback_("websocket client connected");

      boost::beast::flat_buffer buffer;
      while (running_) {
        buffer.clear();
        boost::system::error_code ec;
        websocket->read(buffer, ec);
        if (ec) {
          if (ec != boost::beast::websocket::error::closed && running_) {
            event_callback_("websocket client disconnected or timed out: " + ec.message());
          }
          break;
        }
        message_callback_(boost::beast::buffers_to_string(buffer.data()));
      }
    } catch (const std::exception& exception) {
      if (running_) {
        event_callback_("websocket session error: " + std::string(exception.what()));
      }
    }

    {
      std::lock_guard<std::mutex> lock(sessions_mutex_);
      sessions_.erase(std::remove(sessions_.begin(), sessions_.end(), websocket), sessions_.end());
    }
    event_callback_("websocket client disconnected");
  }

  std::string host_;
  uint16_t port_;
  int idle_timeout_ms_;
  MessageCallback message_callback_;
  EventCallback event_callback_;
  std::atomic_bool running_{false};
  boost::asio::io_context io_context_;
  std::unique_ptr<Tcp::acceptor> acceptor_;
  std::vector<std::shared_ptr<Websocket>> sessions_;
  std::vector<SessionThread> session_threads_;
  std::mutex sessions_mutex_;
  std::thread server_thread_;
};

WebsocketServer::WebsocketServer(std::string host, uint16_t port, int idle_timeout_ms,
                                 MessageCallback message_callback,
                                 EventCallback event_callback)
    : impl_(std::make_unique<Impl>(std::move(host), port, idle_timeout_ms,
                                   std::move(message_callback),
                                   std::move(event_callback))) {}

WebsocketServer::~WebsocketServer() = default;

bool WebsocketServer::start() { return impl_->start(); }

void WebsocketServer::stop() { impl_->stop(); }

RemoteControlNode::RemoteControlNode()
    : Node("remote_control"),
      stop_pid_(StopPidConfig{
          declare_parameter<double>("ebs_pid_kp", 0.0002),
          declare_parameter<double>("ebs_pid_ki", 0.0),
          declare_parameter<double>("ebs_pid_kd", 0.0),
          declare_parameter<double>("ebs_pid_output_min", -0.4),
          declare_parameter<double>("ebs_pid_output_max", 0.4),
          declare_parameter<double>("ebs_pid_integral_min", -1.0),
          declare_parameter<double>("ebs_pid_integral_max", 1.0),
      }) {
  websocket_host_ = declare_parameter<std::string>("websocket_host", "0.0.0.0");
  websocket_port_ = declare_parameter<int>("websocket_port", 4321);
  websocket_idle_timeout_ms_ = declare_parameter<int>("websocket_idle_timeout_ms", 0);
  stale_timeout_ms_ = declare_parameter<int>("stale_timeout_ms", 500);
  command_period_ms_ = declare_parameter<int>("command_period_ms", 25);
  steering_min_rad_ = declare_parameter<double>("steering_min_rad", -0.335);
  steering_max_rad_ = declare_parameter<double>("steering_max_rad", 0.335);
  throttle_min_ = declare_parameter<double>("throttle_min", -1.0);
  throttle_max_ = declare_parameter<double>("throttle_max", 1.0);
  velocity_error_min_ = declare_parameter<double>("velocity_error_min", -2000.0);
  velocity_error_max_ = declare_parameter<double>("velocity_error_max", 2000.0);
  ebs_stop_rpm_tolerance_ = declare_parameter<double>("ebs_stop_rpm_tolerance", 50.0);

  auto state_qos = rclcpp::QoS(1).transient_local().reliable();
  remote_ebs_pub_ = create_publisher<std_msgs::msg::Bool>("/remote/ebs", state_qos);
  remote_take_control_pub_ =
      create_publisher<std_msgs::msg::Bool>("/remote/take_control", state_qos);
  control_command_pub_ =
      create_publisher<custom_interfaces::msg::ControlCommand>("/control/command", 10);
  motor_rpm_sub_ = create_subscription<custom_interfaces::msg::WheelRPM>(
      "/vehicle/motor_rpm", 10,
      std::bind(&RemoteControlNode::motor_rpm_callback, this, std::placeholders::_1));

  command_timer_ = create_wall_timer(std::chrono::milliseconds(command_period_ms_),
                                     std::bind(&RemoteControlNode::command_timer_callback, this));

  websocket_server_ = std::make_unique<WebsocketServer>(
      websocket_host_, static_cast<uint16_t>(websocket_port_), websocket_idle_timeout_ms_,
      std::bind(&RemoteControlNode::handle_websocket_message, this, std::placeholders::_1),
      std::bind(&RemoteControlNode::handle_websocket_event, this, std::placeholders::_1));
  const bool websocket_listening = websocket_server_->start();

  publish_remote_state(false, false);
  if (websocket_listening) {
    RCLCPP_INFO(get_logger(), "Remote websocket bind address: %s", websocket_host_.c_str());
    RCLCPP_INFO(get_logger(), "Remote websocket port: %d", websocket_port_);
    RCLCPP_INFO(get_logger(), "Remote websocket idle timeout: %d ms", websocket_idle_timeout_ms_);
    if (websocket_host_ == "0.0.0.0" || websocket_host_ == "::") {
      const auto addresses = local_ipv4_addresses();
      if (addresses.empty()) {
        RCLCPP_INFO(get_logger(), "Remote websocket URL: ws://<this-car-ip>:%d", websocket_port_);
      } else {
        for (const auto& address : addresses) {
          RCLCPP_INFO(get_logger(), "Remote websocket URL: ws://%s:%d", address.c_str(),
                      websocket_port_);
        }
      }
    } else {
      RCLCPP_INFO(get_logger(), "Remote websocket URL: ws://%s:%d", websocket_host_.c_str(),
                  websocket_port_);
    }
    RCLCPP_INFO(get_logger(), "Remote control receiver initialized");
  } else {
    RCLCPP_ERROR(get_logger(), "Remote control receiver failed to bind websocket %s:%d",
                 websocket_host_.c_str(), websocket_port_);
  }
}

RemoteControlNode::~RemoteControlNode() {
  if (websocket_server_) {
    websocket_server_->stop();
  }
}

void RemoteControlNode::handle_websocket_message(const std::string& message) {
  RCLCPP_INFO(get_logger(), "Received websocket payload: %s", message.c_str());

  const ParseResult parse_result = parse_emergency_control_message(message);
  if (!parse_result.valid) {
    RCLCPP_WARN(get_logger(), "Ignoring malformed remote payload: %s", parse_result.error.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.payload = parse_result.payload;
  state_.last_message_time = now();
  state_.has_message = true;
  state_.stale = false;
  if (state_.payload.e_ebs) {
    state_.ebs_latched = true;
  } else {
    state_.ebs_latched = false;
    stop_pid_.reset();
    state_.has_pid_time = false;
  }
}

void RemoteControlNode::handle_websocket_event(const std::string& event) {
  RCLCPP_INFO(get_logger(), "%s", event.c_str());
}

void RemoteControlNode::motor_rpm_callback(const custom_interfaces::msg::WheelRPM& msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.motor_rpm = (static_cast<double>(msg.rl_rpm) + static_cast<double>(msg.rr_rpm)) / 2.0;
}

void RemoteControlNode::command_timer_callback() {
  State snapshot;
  const rclcpp::Time current_time = now();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_.has_message) {
      const double age_ms = (current_time - state_.last_message_time).seconds() * 1000.0;
      state_.stale = age_ms > static_cast<double>(stale_timeout_ms_);
    }
    snapshot = state_;
  }

  const bool active_ebs = snapshot.ebs_latched;
  const bool active_take_control =
      !active_ebs && snapshot.has_message && snapshot.payload.take_control;
  publish_remote_state(active_ebs, active_take_control);
  log_state_transition(active_ebs, active_take_control, snapshot.stale);

  if (active_ebs) {
    publish_ebs_command(snapshot, current_time);
    return;
  }

  if (active_take_control) {
    publish_manual_command(snapshot.payload);
  }
}

void RemoteControlNode::publish_remote_state(bool ebs, bool take_control) {
  std_msgs::msg::Bool ebs_msg;
  ebs_msg.data = ebs;
  remote_ebs_pub_->publish(ebs_msg);

  std_msgs::msg::Bool take_control_msg;
  take_control_msg.data = take_control;
  remote_take_control_pub_->publish(take_control_msg);
}

void RemoteControlNode::publish_manual_command(const EmergencyControlPayload& payload) {
  const double steering =
      clamp_control_value(payload.steering_rad, steering_min_rad_, steering_max_rad_);
  const double throttle = clamp_control_value(payload.throttle, throttle_min_, throttle_max_);
  control_command_pub_->publish(make_control_command(steering, throttle));
}

void RemoteControlNode::publish_ebs_command(State& state, const rclcpp::Time& current_time) {
  if (std::abs(state.motor_rpm) <= ebs_stop_rpm_tolerance_) {
    control_command_pub_->publish(make_control_command(0.0, 0.0));
    return;
  }

  const double error = clamp_control_value(-state.motor_rpm, velocity_error_min_, velocity_error_max_);

  double dt_seconds = static_cast<double>(command_period_ms_) / 1000.0;
  if (state.has_pid_time) {
    dt_seconds = (current_time - state.last_pid_time).seconds();
  }
  state.last_pid_time = current_time;
  state.has_pid_time = true;

  const double brake_command = stop_pid_.update(error, dt_seconds);
  control_command_pub_->publish(make_control_command(0.0, brake_command));

  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.last_pid_time = state.last_pid_time;
  state_.has_pid_time = state.has_pid_time;
}

void RemoteControlNode::log_state_transition(bool ebs, bool take_control, bool stale) {
  if (ebs != last_published_ebs_ || take_control != last_published_take_control_ ||
      stale != last_logged_stale_) {
    RCLCPP_INFO(get_logger(), "Remote state: ebs=%s take_control=%s stale=%s",
                ebs ? "true" : "false", take_control ? "true" : "false",
                stale ? "true" : "false");
    last_published_ebs_ = ebs;
    last_published_take_control_ = take_control;
    last_logged_stale_ = stale;
  }
}

}  // namespace remote_control
