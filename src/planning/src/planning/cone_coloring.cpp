#include "planning/cone_coloring.hpp"
#include <iostream>

ConeColoring::ConeColoring(double gain_angle, double gain_distance, double gain_ncones,
 double exponent_1, double exponent_2, double cost_max ) : angle_gain(gain_angle),
 distance_gain(gain_distance), ncones_gain(gain_ncones), exponent1(exponent_1),
 exponent2(exponent_2), max_cost(cost_max) {}

double ConeColoring::angle_and_norm(Cone *possible_next_cone, bool side, double &norm) {
    std::vector<Cone *> current_cones = side ? current_left_cones : current_right_cones;
    int n = current_cones.size();
    double x_last = current_cones[n-1] -> getX() - current_cones[n-2] -> getX();
    double y_last = current_cones[n-1] -> getY() - current_cones[n-2] -> getY();
    double norm2 = sqrt(pow(x_last,2) + pow(y_last,2));
    double x_next = possible_next_cone -> getX() - current_cones[n-1] -> getX();
    double y_next = possible_next_cone -> getY() - current_cones[n-1] -> getY();
    norm = sqrt(pow(x_next,2) + pow(y_next,2));
    double dot_product = x_last*x_next + y_last*y_next;
    return acos(dot_product/(norm*norm2));
}

bool ConeColoring::cone_is_in_side(Cone *c, bool side) {
    std::vector<Cone *> current_cones = side ? current_left_cones : current_right_cones;
    for (Cone *cone : current_cones) {
        if (cone -> getId() == c -> getId()) return true;
    }
    return false;
}

double ConeColoring::distance_to_side(Pose initial_car_pose, bool side, double x, double y) {
    if (side) return fabs(y - (initial_car_pose.position.y + 2*cos(initial_car_pose.orientation)) -
     tan(initial_car_pose.orientation)*( x - ( initial_car_pose.position.x - 2*sin(initial_car_pose.orientation))));
    else return fabs(y - (initial_car_pose.position.y - 2*cos(initial_car_pose.orientation)) -
     tan(initial_car_pose.orientation)*( x - ( initial_car_pose.position.x + 2*sin(initial_car_pose.orientation))));
}

Cone* ConeColoring::getInitialCone(std::vector<Cone *> candidates, Pose initial_car_pose, bool side) {
    double minimum_cost = MAXFLOAT;
    Cone *initial_cone = nullptr;
    for (Cone *c : candidates) {
        double distance_to_car = sqrt(pow(initial_car_pose.position.x - c -> getX(), 2) + pow(initial_car_pose.position.y - c -> getY(), 2));
        double cost = distance_to_car * distance_to_side(initial_car_pose, side, c -> getX(), c -> getY());
        if (cost < minimum_cost) {
            minimum_cost = cost;
            initial_cone = c;
        }
    }
    return initial_cone;
}

std::vector<Cone *> ConeColoring::filter_cones_by_distance(std::vector<Cone *> input_cones, Position position, double radius) {
    std::vector<Cone *> filtered_cones;
    for (Cone *c : input_cones) {
        double squared_distance = pow(position.x - c -> getX(), 2) + pow(position.y - c -> getY(), 2);
        if (squared_distance <= radius*radius) filtered_cones.push_back(c);
    }
    return filtered_cones;
}

void ConeColoring::place_initial_cones(std::vector<Cone *> input_cones, Pose initial_car_pose) {
    // filter cones that are farther away than 5 meters
    std::vector<Cone *> candidates = filter_cones_by_distance(input_cones, initial_car_pose.position, 5);

    // get the initial cone for the left side
    Cone *initial_cone_left = getInitialCone(candidates, initial_car_pose, true);

    // get the initial cone for the right side 
    Cone *initial_cone_right = getInitialCone(candidates, initial_car_pose, false);

    // Create virtual cones to be added at the beginning of the vector. They must be dismissed later.
    Cone *virtual_left_cone = new Cone(-2, initial_cone_left->getX() - 2*cos(initial_car_pose.orientation), initial_cone_left -> getY() - 2*sin(initial_car_pose.orientation)); 
    Cone *virtual_right_cone = new Cone(-1, initial_cone_right->getX() - 2*cos(initial_car_pose.orientation), initial_cone_right -> getY() - 2*sin(initial_car_pose.orientation)); 

    current_left_cones.push_back(virtual_left_cone);
    current_right_cones.push_back(virtual_right_cone);
    current_left_cones.push_back(initial_cone_left);
    current_right_cones.push_back(initial_cone_right);
}

double ConeColoring::cost(Cone* possible_next_cone, int n, bool side) {
    std::vector<Cone *> current_cones = side ? current_left_cones : current_right_cones;
    double norm;
    double angle = angle_and_norm(possible_next_cone, side, norm);
    double cost = pow(pow(distance_gain*norm + angle_gain*angle, exponent1) + ncones_gain*current_cones.size()/(float)n, exponent2);
    return cost;
}

std::vector<std::pair<Cone *, bool> *> ConeColoring::makeUnvisitedCones(std::vector<Cone *> cones) {
    std::vector<std::pair<Cone *, bool> *> conePairs;
    for (Cone *cone : cones) {
        std::pair<Cone *, bool> *conePair = new std::pair<Cone *, bool>(cone, cone_is_in_side(cone, true) || cone_is_in_side(cone, false));
        conePairs.push_back(conePair);
    }
    return conePairs;
}

bool ConeColoring::placeNextCone(std::vector<std::pair<Cone *, bool> *> &visited_cones, double distance_threshold, int n, bool side) {
    
    double minimum_cost = MAXFLOAT;
    
    std::vector<Cone *> &current_cones = side ? current_left_cones : current_right_cones;

    Cone *last_cone = current_cones.back();

    Cone *next_cone = visited_cones[0]->first;

    std::pair<Cone *, bool> *best_pair = visited_cones[0];

    for (std::pair<Cone *, bool>* p : visited_cones) {
        if (p -> second) continue; // skip visited cones
        double cost = ConeColoring::cost(p -> first, n, side);
        if (cost < minimum_cost && last_cone->squaredDistanceTo(p ->first) < distance_threshold*distance_threshold && cost < max_cost) {
            minimum_cost = cost;
            next_cone = p -> first;
            best_pair = p;
        }
    }
    if (minimum_cost == MAXFLOAT) return false;
    current_cones.push_back(next_cone);
    best_pair -> second = true;
    return true;
}

void ConeColoring::colorCones(std::vector<Cone *> input_cones, Pose initial_car_pose, double distance_threshold) {
    place_initial_cones(input_cones, initial_car_pose);
    std::vector<std::pair<Cone *, bool> *> visited_cones = makeUnvisitedCones(input_cones);

    while (placeNextCone(visited_cones, distance_threshold, input_cones.size(), true));
    while (placeNextCone(visited_cones, distance_threshold, input_cones.size(), false));
}
