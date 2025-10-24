#include "common_lib/structures/midpoint.hpp"

namespace common_lib::structures {

// Constructor from a Point
Midpoint::Midpoint(const Point& p,
                    std::shared_ptr<Cone> c1,
                    std::shared_ptr<Cone> c2)
    : point(p), close_points(std::vector<std::shared_ptr<Midpoint>>{}), cone1(c1), cone2(c2), valid(true) {}



}  // namespace common_lib::structures
