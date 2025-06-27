#pragma once

#include <cone_evaluator/cone_evaluator.hpp>
#include <cone_validator/cylinder_validator.hpp>
#include <cone_validator/deviation_validator.hpp>
#include <cone_validator/z_score_validator.hpp>
#include <string>
#include <unordered_map>
#include <utils/plane.hpp>
#include <utils/trimming_parameters.hpp>
#include <vector>

#include "clustering/dbscan.hpp"
#include "common_lib/competition_logic/mission_logic.hpp"
#include "cone_differentiation/least_squares_differentiation.hpp"
#include "cone_validator/height_validator.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/master_log.hpp"
#include "fov_trimming/acceleration_trimming.hpp"
#include "fov_trimming/cut_trimming.hpp"
#include "fov_trimming/skidpad_trimming.hpp"
#include "ground_removal/grid_ransac.hpp"
#include "ground_removal/ransac.hpp"
#include "icp/icp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
#include "track_filtering/track_filter.hpp"
#include "utils/precone.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using Mission = common_lib::competition_logic::Mission;
struct PerceptionParameters {     ///< Struct containing parameters and interfaces used in
                                  ///< perception.
  std::string vehicle_frame_id_;  ///< String for the vehicle's frame id.
  std::string adapter_;           ///< String for the name of the current adapter.
  uint8_t default_mission_;
  std::shared_ptr<std::unordered_map<uint8_t, std::shared_ptr<FovTrimming>>> fov_trim_map_;
  std::shared_ptr<GroundRemoval> ground_removal_;  ///< Shared pointer to the GroundRemoval object.
  std::shared_ptr<DBSCAN> clustering_;             ///< Shared pointer to the DBSCAN object.
  std::shared_ptr<LeastSquaresDifferentiation>
      cone_differentiator_;  ///< Shared pointer to ConeDifferentiation object.
  std::shared_ptr<ConeEvaluator> cone_evaluator_;  ///< Shared pointer to ConeEvaluator object.
  std::unordered_map<std::string, double>
      weight_values;           ///< Map containing all weight value parameters for cone evaluation.
  bool track_filter_enabled_;  ///< Boolean to enable or disable the track filter.
  std::shared_ptr<TrackFilter> track_filter_;
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
private:
  std::string _vehicle_frame_id_;  ///< String for the vehicle's frame id.
  std::string _adapter_;           ///< String for the current adapter being used.
  uint8_t _mission_type_;          ///< integer value for the current mission type running.
  Plane _ground_plane_;            ///< Model for the ground plane.
  std::shared_ptr<std::unordered_map<uint8_t, std::shared_ptr<FovTrimming>>>
      _fov_trim_map_;                               ///< Shared pointer to the FovTrimming object.
  std::shared_ptr<GroundRemoval> _ground_removal_;  ///< Shared pointer to the GroundRemoval object.
  std::shared_ptr<Clustering> _clustering_;         ///< Shared pointer to the Clustering object.
  std::shared_ptr<ConeDifferentiation>
      _cone_differentiator_;  ///< Shared pointer to ConeDifferentiation object.
  std::shared_ptr<ConeEvaluator> _cone_evaluator_;  ///< Shared pointer to ConeEvaluator object.
  bool _track_filter_enabled_;  ///< Boolean to enable or disable the track filter.
  std::shared_ptr<TrackFilter> _track_filter_;
  std::shared_ptr<ICP> _icp_;  ///< Shared pointer to ICP object.
  rclcpp::Subscription<custom_interfaces::msg::MasterLog>::SharedPtr
      _master_log_subscription;  ///< Master Log subscription to aquire mission type
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      _point_cloud_subscription;  ///< PointCloud2 subscription.
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr
      _cones_publisher;  ///< ConeArray publisher.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      _cone_marker_array_;  ///< MarkerArray publisher.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      _perception_execution_time_publisher_;  ///< Perception execution time publisher.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      _ground_removed_publisher_;  ///< point cloud after ground removal publisher.

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
  void publish_cones(std::vector<PreCone>* cones);

  std::vector<PreCone> convert_clusters_to_precones(std::vector<Cluster>& clusters);

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
};
