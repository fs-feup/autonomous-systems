#pragma once

#include <cone_validator/cone_validator.hpp>
#include <utils/cluster.hpp>

class ZScoreValidator : public ConeValidator {
 private:
  double _min_z_score_x_;
  double _max_z_score_x_;
  double _min_z_score_y_;
  double _max_z_score_y_;

 public:

  ZScoreValidator(double min_z_score_x, double max_z_score_x, double min_z_score_y, double max_z_score_y);

  bool coneValidator(Cluster* cluster, Plane& plane) const override;
};

