#include "utils/precone.hpp"

PreCone::PreCone(double x, double y, bool is_large, double confidence)
    : position_(common_lib::structures::Position(x, y)),
      is_large_(is_large),
      confidence_(confidence) {}

common_lib::structures::Position PreCone::get_position() const { return position_; }

double PreCone::get_confidence() const { return confidence_; }

bool PreCone::get_is_large() const { return is_large_; }