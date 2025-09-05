#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "slam_solver/graph_slam_solver/optimizer/base_optimizer.hpp"

// inline long long q(double x, double res = 1e-4) {  // tune to your sensor precision
//   return static_cast<long long>(std::llround(x / res));
// }

// struct FactorSet {
//   std::unordered_set<std::string> seen;

//   // --- signature builders (tiny + robust enough) ---
//   static std::string sigBetween(const gtsam::BetweenFactor<gtsam::Pose2>& f, double res = 1e-4) {
//     auto a = f.keys()[0], b = f.keys()[1];
//     if (a > b) std::swap(a, b);    // order-independent
//     const auto& m = f.measured();  // x, y, theta
//     std::ostringstream ss;
//     ss << "BetweenPose2|" << a << "|" << b << "|" << q(m.x(), res) << "," << q(m.y(), res) << ","
//        << q(m.theta(), res);
//     return ss.str();
//   }

//   static std::string sigBR(const gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>& f,
//                            double res = 1e-4) {
//     auto a = f.keys()[0], b = f.keys()[1];  // pose key, landmark key
//     // For BearingRange the order matters (pose->landmark), so no sort here.
//     gtsam::BearingRange<gtsam::Pose2, gtsam::Point2> br = f.measured();  // pair<Rot2, double>
//     std::ostringstream ss;
//     ss << "BearingRange2|" << a << "|" << b << "|" << q(br.bearing().theta(), res) << ","
//        << q(br.range(), res);
//     return ss.str();
//   }

//   // --- API: alreadyAdded / add (overloads for each factor type) ---
//   bool alreadyAdded(const boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2>>& f,
//                     double res = 1e-4) const {
//     return seen.count(sigBetween(*f, res)) > 0;
//   }
//   void add(const boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2>>& f, double res = 1e-4) {
//     seen.insert(sigBetween(*f, res));
//   }

//   bool alreadyAdded(
//       const boost::shared_ptr<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>>& f,
//       double res = 1e-4) const {
//     return seen.count(sigBR(*f, res)) > 0;
//   }
//   void add(const boost::shared_ptr<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>>& f,
//            double res = 1e-4) {
//     seen.insert(sigBR(*f, res));
//   }

//   void clear() { seen.clear(); }
// };

class ISAM2Optimizer : public BaseOptimizer {
  gtsam::ISAM2 _isam2_;                       //< ISAM2 instance for the optimizer
  gtsam::Values _last_estimate_;              //< Last estimate from the optimizer
  gtsam::Values _new_values_;                 //< New values to add to the optimizer
  gtsam::NonlinearFactorGraph _new_factors_;  //< New factors to add to the optimizer
public:
  ISAM2Optimizer(const SLAMParameters& params);

  ~ISAM2Optimizer() override = default;

  gtsam::Values optimize(gtsam::NonlinearFactorGraph& factor_graph, gtsam::Values& graph_values,
                         [[maybe_unused]] unsigned int pose_num,
                         [[maybe_unused]] unsigned int landmark_num) override;

  friend class GraphSLAMInstance;
};