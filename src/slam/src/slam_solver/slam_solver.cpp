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
