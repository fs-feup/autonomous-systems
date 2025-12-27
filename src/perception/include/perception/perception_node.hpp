#pragma once

#include <cone_evaluator/cone_evaluator.hpp>
#include <cone_validator/cylinder_validator.hpp>
#include <cone_validator/deviation_validator.hpp>
#include <cone_validator/displacement_validator.hpp>
#include <cone_validator/npoints_validator.hpp>
#include <cone_validator/z_score_validator.hpp>
#include <string>
#include <unordered_map>
#include <utils/ground_grid.hpp>
#include <utils/plane.hpp>
#include <utils/trimming_parameters.hpp>
#include <vector>

#include "clustering/dbscan.hpp"
#include "clustering/grid_clustering.hpp"
#include "common_lib/communication/marker.hpp"
#include "common_lib/competition_logic/mission_logic.hpp"
#include "common_lib/config_load/config_load.hpp"
#include "common_lib/structures/velocities.hpp"
#include "cone_differentiation/least_squares_differentiation.hpp"
#include "cone_validator/height_validator.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/perception_output.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "deskew/deskew.hpp"
#include "fov_trimming/acceleration_trimming.hpp"
#include "fov_trimming/cut_trimming.hpp"
#include "fov_trimming/skidpad_trimming.hpp"
#include "ground_removal/himmelsbach.hpp"
#include "ground_removal/ransac.hpp"
#include "icp/icp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "wall_removal/grid_removal.hpp"
#include "wall_removal/wall_removal.hpp"
#include "yaml-cpp/yaml.h"

using Mission = common_lib::competition_logic::Mission;

/**
 * @brief Struct containing parameters and interfaces used in perception.
 */
struct PerceptionParameters {
  std::string vehicle_frame_id_;  ///< String for the vehicle's frame id.
  std::string adapter_;           ///< String for the name of the current adapter.
  uint8_t default_mission_;
  bool publish_fov_trimmed_cloud;
  bool publish_ground_removed_cloud;
  bool publish_wall_removed_cloud;
  std::shared_ptr<std::unordered_map<int16_t, std::shared_ptr<FovTrimming>>> fov_trim_map_;
  std::shared_ptr<GroundGrid> ground_grid_;        ///< Model for the ground grid.
  std::shared_ptr<GroundRemoval> ground_removal_;  ///< Shared pointer to the GroundRemoval object.
  std::shared_ptr<WallRemoval> wall_removal_;      ///< Shared pointer to the WallRemoval object.
  std::shared_ptr<Clustering> clustering_;         ///< Shared pointer to the Clustering object.
  std::shared_ptr<LeastSquaresDifferentiation>
      cone_differentiator_;  ///< Shared pointer to ConeDifferentiation object.
  std::shared_ptr<ConeEvaluator> cone_evaluator_;  ///< Shared pointer to ConeEvaluator object.
  std::unordered_map<std::string, double>
      weight_values;          ///< Map containing all weight value parameters for cone evaluation.
  std::shared_ptr<ICP> icp_;  ///< Shared pointer to ICP object.
};

/**
 * @class Perception
 * @brief Node for perception tasks, such as ground removal and cone detection.
 *
 * This class is a ROS 2 node that subscribes to a PointCloud2 topic, performs
 * ground removal using the specified GroundRemoval algorithm, and publishes
 * cone detection results on a custom topic.
 */
class Perception : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the Perception node.
   * @param params The parameters for perception.
   */
  explicit Perception(const PerceptionParameters& params);

  /**
   * @brief Turns the parameters in the yaml file into PerceptionParameters class.
   * @return parameters configured following yaml file.
   *
   */
  static PerceptionParameters load_config();

  /**
   * @brief Callback function for the PointCloud2 subscription.
   * @param msg The received PointCloud2 message.
   */
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  std::string _vehicle_frame_id_;       ///< String for the vehicle's frame id.
  std::string _adapter_;                ///< String for the current adapter being used.
  int16_t _mission_type_;               ///< integer value for the current mission type running.
  bool _publish_fov_trimmed_cloud_;     ///< Whether to publish the FOV trimmed point cloud.
  bool _publish_ground_removed_cloud_;  ///< Whether to publish the ground removed point cloud.
  bool _publish_wall_removed_cloud_;    ///< Whether to publish the wall removed point cloud.
  std::shared_ptr<std::unordered_map<int16_t, std::shared_ptr<FovTrimming>>>
      _fov_trim_map_;                               ///< Shared pointer to the FovTrimming object.
  std::shared_ptr<GroundGrid> _ground_grid_;        ///< Model for the ground grid.
  std::shared_ptr<GroundRemoval> _ground_removal_;  ///< Shared pointer to the GroundRemoval object.
  std::shared_ptr<WallRemoval> _wall_removal_;      ///< Shared pointer to the WallRemoval object.
  std::shared_ptr<Clustering> _clustering_;         ///< Shared pointer to the Clustering object.
  std::shared_ptr<ConeDifferentiation>
      _cone_differentiator_;  ///< Shared pointer to ConeDifferentiation object.
  std::shared_ptr<ConeEvaluator> _cone_evaluator_;  ///< Shared pointer to ConeEvaluator object.
  std::shared_ptr<ICP> _icp_;                       ///< Shared pointer to ICP object.
  std::shared_ptr<Deskew> _deskew_;                 ///< Shared pointer to Deskew object.
  rclcpp::Subscription<custom_interfaces::msg::OperationalStatus>::SharedPtr
      _operational_status_subscription;  ///< Master Log subscription to aquire mission type
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      _point_cloud_subscription;  ///< PointCloud2 subscription.
  rclcpp::Subscription<custom_interfaces::msg::Velocities>::SharedPtr _velocities_subscription_;
  rclcpp::Publisher<custom_interfaces::msg::PerceptionOutput>::SharedPtr
      _cones_publisher;  ///< ConeArray + exec publisher.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      _cone_marker_array_;  ///< MarkerArray publisher.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      _perception_execution_time_publisher_;  ///< Perception execution time publisher.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      _fov_trimmed_publisher_;  ///< Point cloud after fov trimming publisher.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      _ground_removed_publisher_;  ///< Point cloud after ground removal publisher.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      _wall_removed_publisher_;  ///< Point cloud after wall removal publisher.
  common_lib::structures::Velocities
      _vehicle_velocity_;  // Last received vehicle velocity, used to deskew the point cloud

  rclcpp::TimerBase::SharedPtr lidar_off_timer_;  ///< Timer to check for lidar emergency
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_client_;
  std::shared_ptr<std::vector<double>> _execution_times_;  ///< Vector to store execution times.

  /**
   * @brief Publishes information about clusters (cones) using a custom ROS2 message.
   *
   * This function takes a vector of Cluster objects, extracts relevant information such as
   * centroid and color, and publishes this information using a custom ROS2 message type
   * ConeArray.
   *
   * @param cones A reference to a vector of Cluster objects representing the clusters (cones)
   * to be published.
   */
  void publish_cones(std::vector<Cluster>* cones, double exec_time, rclcpp::Time start_time);

  /**
   * @brief Callback function for the Velocities subscription
   * @param msg The received Velocities message.
   *
   */
  void velocities_callback(const custom_interfaces::msg::Velocities& msg);

  /**
   * @brief Callback function for the LiDAR timer to check for emergency
   */
  void lidar_timer_callback();
};
