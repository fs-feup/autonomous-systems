#include "slam_solver/slam_solver.hpp"

SLAMSolver::SLAMSolver(const SLAMParameters& params,
                       std::shared_ptr<DataAssociationModel> data_association,
                       std::shared_ptr<V2PMotionModel> motion_model,
                       std::shared_ptr<std::vector<double>> execution_times,
                       std::weak_ptr<rclcpp::Node> node)
    : _params_(params),
      _data_association_(data_association),
      _motion_model_(motion_model),
      _execution_times_(execution_times),
      _node_(node) {}

SLAMSolver::SLAMSolver(const SLAMParameters& params,
                       std::shared_ptr<DataAssociationModel> data_association,
                       std::shared_ptr<V2PMotionModel> motion_model,
                       std::shared_ptr<std::vector<double>> execution_times)
    : _params_(params),
      _data_association_(data_association),
      _motion_model_(motion_model),
      _execution_times_(execution_times) {}