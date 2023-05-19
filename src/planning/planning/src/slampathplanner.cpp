#include "../include/planning/slampathplanner.hpp"

SlamPathPlanner::SlamPathPlanner(){
    track = new Track();
}

vector<Position*> SlamPathPlanner::processNewArray(Track* cone_array){
    std::cout << "Sizes: " << cone_array->getLeftConesSize() << " " << cone_array->getRightConesSize() << "\n";
    vector<Position*> path; 
    for (int i = 0; i < cone_array->getLeftConesSize(); i++){
        this->track->setCone(cone_array->getLeftConeAt(i));
        //std::cout << this->track->getLeftConesSize();
    }
        
    for (int i = 0; i < cone_array->getRightConesSize(); i++)
        this->track->setCone(cone_array->getRightConeAt(i));

    
    //delete(cone_array);
        
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
        dt.insert(Point(lCone->getX(), lCone->getY()));
        std::cout << lCone->getX();
    }
          
    for (int i = 0; i < track->getRightConesSize(); i++){
        Cone* rCone = track->getRightConeAt(i);
        dt.insert(Point(rCone->getX(), rCone->getY()));
    }

    for (DT::Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
        float x1 = it->first->vertex((it->second + 1) % 3)->point().x();
        float y1 = it->first->vertex((it->second + 1) % 3)->point().y();
        float x2 = it->first->vertex((it->second + 2) % 3)->point().x();
        float y2 = it->first->vertex((it->second + 2) % 3)->point().y();

        Cone* cone1 = track->findCone(x1, y1);
        Cone* cone2 = track->findCone(x2, y2);

        if (cone1 != nullptr && cone2 != nullptr && cone1->getId() % 2 != cone2->getId() % 2){
            float xDist = cone2->getX() - cone1->getX();
            float yDist = cone2->getY() - cone1->getY();

            Position* position = new Position(cone1->getX() + xDist / 2, cone1->getY() + yDist / 2);
            path.push_back(position);          
        }
    }

    return path;
}