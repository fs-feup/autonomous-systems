#include "adapter_planning/eufs.hpp"
#include "adapter_planning/fsds.hpp"
#include "adapter_planning/pacsim.hpp"
#include "adapter_planning/vehicle.hpp"
#include "planning/planning.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  std::string adapter;
  PlanningParameters params = Planning::load_config(adapter);

  std::shared_ptr<Planning> planning;
  if (adapter == "vehicle") {
    planning = std::make_shared<VehicleAdapter>(params);
  } else if (adapter == "pacsim") {
    planning = std::make_shared<PacSimAdapter>(params);
  } else if (adapter == "eufs") {
    planning = std::make_shared<EufsAdapter>(params);
  } else if (adapter == "fsds") {
    planning = std::make_shared<FsdsAdapter>(params);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Adapter type not recognized");
    return 1;
  }

  rclcpp::spin(planning);
  rclcpp::shutdown();
  return 0;
}