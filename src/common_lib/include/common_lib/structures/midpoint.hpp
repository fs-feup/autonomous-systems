#pragma once

#include <functional>

#include "common_lib/structures/cone.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_2.h>
#include <iostream>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = K::Point_2;

namespace common_lib::structures {
/**
 * @brief MidPoint struct represents a potential path point with connections
 */
struct Midpoint {
    Point point;
    std::vector<std::shared_ptr<Midpoint>> close_points; 
    std::shared_ptr<Cone> cone1;
    std::shared_ptr<Cone> cone2;
    bool valid = true;

    Midpoint() = default; 
    Midpoint(const Point& p,
        std::shared_ptr<Cone> c1,
        std::shared_ptr<Cone> c2);
};

}  // namespace common_lib::structures

