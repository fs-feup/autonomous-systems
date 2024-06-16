#include "common_lib/structures/landmark.hpp"

namespace common_lib::structures {

Landmark::Landmark(Position position, int numObservations)
    : position(std::move(position)), numObservations(numObservations) {}

Landmark::Landmark(Position position) : position(std::move(position)) {}

}  // namespace common_lib::structures