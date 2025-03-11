#include "slam_solver/slam_solver.hpp"

SLAMSolver::SLAMSolver(const SLAMParameters& params,
                       std::shared_ptr<DataAssociationModel> data_association,
                       std::shared_ptr<V2PMotionModel> motion_model)
    : _params_(params), _data_association_(data_association), _motion_model_(motion_model) {}
