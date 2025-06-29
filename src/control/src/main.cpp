#include <memory>
#include <string>

#include "adapter_control/eufs.hpp"
#include "adapter_control/fsds.hpp"
#include "adapter_control/pacsim.hpp"
#include "adapter_control/vehicle.hpp"
#include "node_/node_control.hpp"
#include "rclcpp/rclcpp.hpp"



int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  std::string adapter;
  ControlParameters params = Control::load_config(adapter);

  std::shared_ptr<Control> control;
  if (adapter == "vehicle") {
    control = std::make_shared<VehicleAdapter>(params);
  } else if (adapter == "pacsim") {
    control = std::make_shared<PacSimAdapter>(params);
  } else if (adapter == "eufs") {
    control = std::make_shared<EufsAdapter>(params);
  } else if (adapter == "fsds") {
    control = std::make_shared<FsdsAdapter>(params);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("control"), "Adapter type not recognized");
    return 1;
  }

  rclcpp::spin(control);
  rclcpp::shutdown();
  return 0;
}