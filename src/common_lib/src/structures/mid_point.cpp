#include "common_lib/structures/mid_point.hpp"

namespace common_lib::structures {

// Constructor from a Point
MidPoint::MidPoint(const Point& p, Cone* c1, Cone* c2, const std::vector<Point>& close_points)
    : point(p), cone1(c1), cone2(c2), close_points(close_points), valid(true) {}


}  // namespace common_lib::structures
