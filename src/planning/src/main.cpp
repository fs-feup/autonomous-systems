#include <gsl/gsl_errno.h>

#include "adapter_planning/map.hpp"
#include "planning/planning.hpp"

int main(int argc, char* argv[]) {
  // GSL's default handler aborts the process; degenerate inputs are routine here.
  ::gsl_set_error_handler_off();

  rclcpp::init(argc, argv);

  std::string adapter;
  PlanningParameters params = Planning::load_config(adapter);

  std::shared_ptr<Planning> planning = adapter_map.at(params.planning_adapter_)(params);

  rclcpp::spin(planning);
  rclcpp::shutdown();
  return 0;
}