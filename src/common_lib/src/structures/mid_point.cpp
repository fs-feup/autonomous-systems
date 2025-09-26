#include "common_lib/structures/mid_point.hpp"

namespace common_lib::structures {

// Constructor from a Point
MidPoint::MidPoint(const Point& p,
                    std::shared_ptr<Cone> c1,
                    std::shared_ptr<Cone> c2)
    : point(p), close_points(std::vector<std::shared_ptr<MidPoint>>{}), cone1(c1), cone2(c2), valid(true) {}



}  // namespace common_lib::structures
