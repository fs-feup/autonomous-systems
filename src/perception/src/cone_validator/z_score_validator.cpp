#include <cone_validator/z_score_validator.hpp>

ZScoreValidator::ZScoreValidator(double min_z_score_x, double max_z_score_x, double min_z_score_y,
                                 double max_z_score_y)
    : _min_z_score_x_(min_z_score_x),
      _max_z_score_x_(max_z_score_x),
      _min_z_score_y_(min_z_score_y),
      _max_z_score_y_(max_z_score_y){};

std::vector<double> ZScoreValidator::coneValidator([[maybe_unused]] Cluster* cluster,
                                                   [[maybe_unused]] Plane& plane) const {
  /*
  (_min_z_score_x_ <= cluster->get_z_score_x() &&
        cluster->get_z_score_x() <= _max_z_score_x_) &&
       (_min_z_score_y_ <= cluster->get_z_score_y() &&
        cluster->get_z_score_y() <= _max_z_score_y_);
  */
  return {};
}