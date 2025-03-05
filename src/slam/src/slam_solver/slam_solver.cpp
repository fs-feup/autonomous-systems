#include "slam_solver/slam_solver.hpp"

SLAMSolver::SLAMSolver(const SLAMSolverParameters& params,
                       const std::shared_ptr<DataAssociationModel> data_association,
                       const std::shared_ptr<V2PMotionModel> motion_model)
    : _params_(params), _data_association_(data_association), _motion_model_(motion_model) {}