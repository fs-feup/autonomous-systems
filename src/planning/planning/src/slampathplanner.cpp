#include "../include/planning/slampathplanner.hpp"

SlamPathPlanner::SlamPathPlanner(){}

void SlamPathPlanner::processNewArray(Track* cone_array){
    for (int i = 0; i < cone_array->getLeftConesSize(); i++)
        this->track->setCone(cone_array->getLeftConeAt(i));

    for (int i = 0; i < cone_array->getRightConesSize(); i++)
        this->track->setCone(cone_array->getRightConeAt(i));

    delete(cone_array);
        
    // only use cones seen in front ignore the other side / very far away cones
    // std::vector<Point> points;
    // points.push_back(Point(0, 0));
    // points.push_back(Point(1.5, 0));
    // points.push_back(Point(0, 1));
    // points.push_back(Point(1.2, 1.7));
    // points.push_back(Point(2, 2));

    DT dt;
    // dt.insert(points.begin(), points.end());
    for (int i = 0; i < track->getLeftConesSize(); i++){
        Cone* lCone = track->getLeftConeAt(i);
        std::cout << lCone->getX() << "\n";
        dt.insert(Point(lCone->getX(), lCone->getY()));
    }
          
    for (int i = 0; i < track->getRightConesSize(); i++){
        Cone* rCone = track->getRightConeAt(i);
        dt.insert(Point(rCone->getX(), rCone->getY()));
    }

    for (DT::Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
        std::cout << "plt.line(" << it->first->vertex((it->second + 1) % 3)->point() << ", " << it->first->vertex((it->second + 2) % 3)->point() << ")" << std::endl;
    }

    //CGAL::draw(dt);

}