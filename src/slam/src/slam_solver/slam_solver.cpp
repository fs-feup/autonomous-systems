#include "slam_solver/slam_solver.hpp"

SLAMSolver::SLAMSolver(const SLAMParameters& params,
                       std::shared_ptr<DataAssociationModel> data_association,
                       std::shared_ptr<V2PMotionModel> motion_model,
                       std::shared_ptr<LandmarkFilter> landmark_filter,
                       std::shared_ptr<std::vector<double>> execution_times,
                       std::shared_ptr<LoopClosure> loop_closure)
    : _params_(params),
      _data_association_(data_association),
      _motion_model_(motion_model),
      _landmark_filter_(landmark_filter),
      _execution_times_(execution_times),
      _loop_closure_(loop_closure) {}

void SLAMSolver::set_mission(common_lib::competition_logic::Mission mission) {
  common_lib::competition_logic::Mission previous_mission_ = this->_mission_;
  _mission_ = mission;
  if (previous_mission_ == common_lib::competition_logic::Mission::NONE &&
      (this->_mission_ == common_lib::competition_logic::Mission::SKIDPAD ||
       this->_mission_ == common_lib::competition_logic::Mission::ACCELERATION) &&
      this->_params_.using_preloaded_map_) {
    if (_mission_ == common_lib::competition_logic::Mission::SKIDPAD) {
      Eigen::Vector3d pose;
      Eigen::VectorXd map;
      load_skidpad_track(pose, map);
      this->load_initial_state(map, pose);
    } else if (_mission_ == common_lib::competition_logic::Mission::ACCELERATION) {
      Eigen::Vector3d pose;
      Eigen::VectorXd map;
      load_acceleration_track(pose, map);
      this->load_initial_state(map, pose);
    }
  }
}
