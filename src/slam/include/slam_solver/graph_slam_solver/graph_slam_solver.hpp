#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <queue>

#include "motion_lib/v2p_models/odometry_model.hpp"
#include "perception_sensor_lib/loop_closure/lap_counter.hpp"
#include "slam_solver/graph_slam_solver/factor_data_structures.hpp"
#include "slam_solver/graph_slam_solver/graph_slam_instance.hpp"
#include "slam_solver/graph_slam_solver/optimizer/base_optimizer.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/base_pose_updater.hpp"
#include "slam_solver/slam_solver.hpp"
#include "slam_solver/solver_traits/imu_integrator_trait.hpp"
#include "slam_solver/solver_traits/node_controller_trait.hpp"
#include "slam_solver/solver_traits/odometry_integrator_trait.hpp"
#include "slam_solver/solver_traits/trajectory_calculator.hpp"
#include "slam_solver/solver_traits/velocities_integrator_trait.hpp"

/**
 * @brief Graph SLAM solver class
 *
 * @details This class implements the Graph SLAM solver using GTSAM
 * It uses a factor graph to represent the problem and the values to store the estimates
 * This specific class controls the access to the models used in the SLAM implementation,
 * including measures for parallel execution
 */
class GraphSLAMSolver : public SLAMSolver,
                        public VelocitiesIntegratorTrait,
                        public OdometryIntegratorTrait,
                        public ImuIntegratorTrait,
                        public NodeControllerTrait,
                        public TrajectoryCalculator {
  std::shared_ptr<GraphSLAMInstance> _graph_slam_instance_;  //< Instance of the graph SLAM solver
  std::shared_ptr<PoseUpdater> _pose_updater_;  //< Pose updater for the graph SLAM solver
  std::shared_ptr<OdometryModel>
      _odometry_model;  //< Motion model for odometry integration TODO: improve, I dont like it
  std::queue<MotionData>
      _motion_data_queue_;  //< Queue of velocities received while optimization ran
  std::queue<ObservationData>
      _observation_data_queue_;  //< Queue of observations received while optimization ran
  rclcpp::TimerBase::SharedPtr _optimization_timer_;  //< Timer for asynchronous optimization
  mutable std::shared_mutex
      _mutex_;  //< Mutex for the graph SLAM solver, locks access to the graph and other vars
  bool _optimization_under_way_ = false;  //< Flag to check if the optimization is under way
  Eigen::VectorXi _associations_;         //< Associations of the cones in the map
  Eigen::VectorXd _observations_global_;  //< Global observations of the cones
  Eigen::VectorXd _map_coordinates_;      //< Coordinates of the landmarks in the map

  common_lib::sensor_data::ImuData _last_imu_data_;  //< Last IMU data received

  rclcpp::CallbackGroup::SharedPtr
      _reentrant_group_;  //< Reentrant callback group for the timer callback

  /**
   * @brief Asynchronous optimization routine
   * @details This method is used to run the optimization in a separate thread
   * It is called by the timer and runs the optimization on the graph
   * It also updates the pose and the graph values accordingly afterwards
   */
  void _asynchronous_optimization_routine();

  /**
   * @brief Adds motion data to the graph if the pose updater is ready
   * @details This method checks if the pose updater has a new pose to add to the graph
   * If so, it adds the new pose to the graph and resets the pose updater's accumulated pose
   * difference
   * @param pose_updater Pose updater to get the new pose from
   * @param graph_slam_instance Graph SLAM instance to add the new pose to
   * @param force_update If true, forces the update even if the pose updater is not ready
   * @return true if the pose was added to the graph, false otherwise
   */
  bool _add_motion_data_to_graph(const std::shared_ptr<PoseUpdater> pose_updater,
                                 const std::shared_ptr<GraphSLAMInstance> graph_slam_instance,
                                 bool force_update = false);

  friend class GraphSlamSolverTest_MotionAndObservation_Test;

public:
  /**
   * @brief Construct a new GraphSLAMSolver object
   *
   * @param params Parameters for the SLAM solver
   * @param data_association Data association module
   * @param motion_model Motion model
   * @param execution_times Timekeeping array
   */
  GraphSLAMSolver(const SLAMParameters& params,
                  std::shared_ptr<DataAssociationModel> data_association,
                  std::shared_ptr<V2PMotionModel> motion_model,
                  std::shared_ptr<LandmarkFilter> landmark_filter,
                  std::shared_ptr<std::vector<double>> execution_times,
                  std::shared_ptr<LoopClosure> loop_closure);

  ~GraphSLAMSolver() = default;

  /**
   * @brief Initialize the SLAM solver
   * @details This method is used to initialize the SLAM solver's
   * aspects that require the node e.g. timer callbacks
   *
   * @param node ROS2 node
   */
  void init(std::weak_ptr<rclcpp::Node> node) override;

  /**
   * @brief Process new velocities data (prediction step)
   * @details It calls the pose updater using the motion model to predict the new pose
   * and may add the motion data to the graph
   * @param velocities Velocities data
   */
  void add_velocities(const common_lib::structures::Velocities& velocities) override;

  /**
   * @brief Add odometry data to the solver (for prediction step)
   * @details It calls the pose updater using the motion model to predict the new pose
   * and may add the motion data to the graph
   * @param pose_difference Pose difference in the form of [dx, dy, dtheta]
   */
  void add_odometry(const common_lib::structures::Pose& pose_difference) override;

  /**
   * @brief Add IMU data to the solver
   *
   * @param imu_data IMU data
   */
  void add_imu_data(const common_lib::sensor_data::ImuData& imu_data) override;

  /**
   * @brief Add observations to the solver (correction step)
   *
   * @param cones Positions of the observations
   */
  void add_observations(const std::vector<common_lib::structures::Cone>& cones) override;

  /**
   * @brief Initialize the graph SLAM solver with a previously saved map and pose
   *
   * @param map Coordinates of the landmarks in the form of [x1, y1, x2, y2, ...] relative to the
   * global frame
   * @param pose Pose of the robot in the form of [x, y, theta] relative to the global frame
   */
  void load_initial_state(const Eigen::VectorXd& map, const Eigen::Vector3d& pose) override;

  /**
   * @brief Get the map estimate object
   *
   * @return std::vector<common_lib::structures::Cone>
   */
  std::vector<common_lib::structures::Cone> get_map_estimate() override;

  /**
   * @brief Get the pose estimate object
   *
   * @return common_lib::structures::Pose
   */
  common_lib::structures::Pose get_pose_estimate() override;

  /**
   * @brief Get the trajectory estimate of the car (all poses)
   *
   * @return std::vector<common_lib::structures::Pose>
   */
  std::vector<common_lib::structures::Pose> get_trajectory_estimate() override;

  /**
   * @brief Get the covariance matrix of the graph
   *
   * @return Eigen::MatrixXd covariance matrix
   */
  Eigen::MatrixXd get_covariance() override;

  /**
   * @brief Get the lap counter
   *
   * @return int lap counter
   */
  int get_lap_counter() override { return lap_counter_; }

  Eigen::VectorXi get_associations() const override;

  Eigen::VectorXd get_observations_global() const override;

  Eigen::VectorXd get_map_coordinates() const override;

  /**
   * Timekeeping array
   * - 0: total motion time (prediction)
   * - 1: total observation time (correction)
   * - 2: data association time
   * - 3: covariance time
   * - 4: factor graph time (in add_observations)
   * - 5: optimization time
   * - 6: optimization copy / initialization time
   * - 7: redo time in optimization routine
   * - 8: total asynchronous optimization routine time
   * - 9: motion model time
   * - 10: factor graph time (in add_velocities)
   * - 11: perception filter time
   * - 12: loop closure time
   * - 13: optimization time in pose update
   */
};