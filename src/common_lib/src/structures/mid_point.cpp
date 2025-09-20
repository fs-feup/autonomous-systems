#include "common_lib/structures/mid_point.hpp"

namespace common_lib::structures {

// Constructor from a Point
MidPoint::MidPoint(const Point& p, Cone* c1, Cone* c2)
    : point(p), cone1(c1), cone2(c2), valid(true) {}


}  // namespace common_lib::structures
