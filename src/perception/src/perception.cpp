#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>

#include <cone_evaluator/distance_predict.hpp>
#include <cstdio>
#include <memory>
#include <vector>

#include "adapter_perception/parameters_factory.hpp"
#include "perception/perception_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  PerceptionParameters params;
  std::string adapter_type = load_adapter_parameters(params);
  std::shared_ptr<Perception> perception = create_perception(adapter_type, params);

  if (!perception) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Failed to create perception object");
    return 1;
  }

  rclcpp::spin(perception);  // Spin the perception node
  rclcpp::shutdown();
  return 0;
}
