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

  void timer_callback() {
    custom_interfaces::msg::VehicleInfo data = this->communicator_->read_from_car();
    publisher_->publish(data);
  }

  void command_callback() { this->communicator_->send_to_car(); }

 public:
  Orchestrator(std::string mode) : Node("orchestrator") {
    if (mode == "ads-dv") {
      this->communicator_ = new AdsDvCommunicator();
    } else if (mode == "eufs") {
      this->communicator_ = new EufsCommunicator(this);
    } else if (mode == "fsds") {
      this->communicator_ = new FsdsCommunicator();
    } else {
      printf("Invalid mode!\r\n");
    }

    publisher_ = this->create_publisher<custom_interfaces::msg::VehicleInfo>("vehicle_info", 10);
    subscriber_ = this->create_subscription<custom_interfaces::msg::VehicleCommand>(
        "vehicle_command", 10,
        std::bind(&Orchestrator::command_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(100ms, std::bind(&Orchestrator::timer_callback, this));
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    printf("Too few arguments!\r\n");
    printf("Usage: ros2 run orchestrator orchestrator <mode>\r\n");
    return 1;
  }

  rclcpp::spin(std::make_shared<Orchestrator>(argv[1]));
  rclcpp::shutdown();

  return 0;
}