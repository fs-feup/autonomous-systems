#include <stdio.h>
#include <unistd.h>

#include "communicators/ads-dv.hpp"
#include "communicators/communicator.hpp"
#include "communicators/eufs.hpp"
#include "communicators/fsds.hpp"
#include "custom_interfaces/msg/vehicle_command.hpp"
#include "custom_interfaces/msg/vehicle_info.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Orchestrator : public rclcpp::Node {
 private:
  rclcpp::Publisher<custom_interfaces::msg::VehicleInfo>::SharedPtr publisher_;
  rclcpp::Subscription<custom_interfaces::msg::VehicleCommand>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  Communicator* communicator_;

  void timer_callback();
  void command_callback();

 public:
  Orchestrator(std::string mode) : Node("orchestrator");
};