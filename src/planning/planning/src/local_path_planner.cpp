#include "../include/planning/local_path_planner.hpp"

LocalPathPlanner::LocalPathPlanner():track() {}

vector<Position*> LocalPathPlanner::processNewArray(Track* cone_array) {
    vector<std::pair<Position*, bool>> unorderedPath;
    for (int i = 0; i < cone_array->getLeftConesSize(); i++)
        this->track.setCone(cone_array->getLeftConeAt(i));

    for (int i = 0; i < cone_array->getRightConesSize(); i++)
        this->track.setCone(cone_array->getRightConeAt(i));

    DT dt;

    for (int i = 0; i < track.getLeftConesSize(); i++) {
        Cone* lCone = track.getLeftConeAt(i);
        dt.insert(Point(lCone->getX(), lCone->getY()));
    }

    for (int i = 0; i < track.getRightConesSize(); i++) {
        Cone* rCone = track.getRightConeAt(i);
        dt.insert(Point(rCone->getX(), rCone->getY()));
    }

    std::map<Cone*, Cone*> connectionMap;
    std::map<Cone*, Cone*> connectionMap2;

    // Select the valid triangulations and add them to the map

    for (DT::Finite_edges_iterator it = dt.finite_edges_begin();
        it != dt.finite_edges_end(); ++it) {        
        float x1 = it->first->vertex((it->second + 1) % 3)->point().x();
        float y1 = it->first->vertex((it->second + 1) % 3)->point().y();
        float x2 = it->first->vertex((it->second + 2) % 3)->point().x();
        float y2 = it->first->vertex((it->second + 2) % 3)->point().y();

        Cone* cone1 = track.findCone(x1, y1);
        Cone* cone2 = track.findCone(x2, y2);
        //std::cout << cone1->getId() << " " << cone2->getId() << "\n";

        if (cone1 != nullptr && cone2 != nullptr
            && cone1->getId() % 2 != cone2->getId() % 2) {   
   
            float xDist = cone2->getX() - cone1->getX();
            float yDist = cone2->getY() - cone1->getY();
            Position* position = new Position(cone1->getX() + xDist / 2, cone1->getY() + yDist / 2);
         
            unorderedPath.push_back(std::make_pair(position, false));
        }        
    }

    vector<Position*> finalPath;

    size_t i = 0;
    size_t iterNumber = 0;
    while (iterNumber < unorderedPath.size()){
        Position* p1 = unorderedPath[i].first;
        float minDist = MAXFLOAT; 
        int minIndex = 0;
        for (size_t j = 0; j < unorderedPath.size(); j++){        
            Position* p2 = unorderedPath[j].first;
            if (unorderedPath[j].second == false && j != i){
                float newDist = p1->getDistanceTo(p2);
                if (newDist < minDist){
                    minDist = newDist;
                    minIndex = j;
                }
            }   
        }
        i = minIndex;
        iterNumber++;
        unorderedPath[minIndex].second = true;
        finalPath.push_back(unorderedPath[minIndex].first);
    }

    
  
    for (size_t i = 0; i < finalPath.size(); i++){
        std::cout << finalPath[i]->getX() << " " << finalPath[i]->getY() << "\n";
    }
    // delete(cone_array);

    return finalPath;
}

float LocalPathPlanner::euclideanDist(Position* p1, Position* p2){
    return sqrt(pow(p2->getX() - p1->getX(), 2) + pow(p2->getY() - p1->getY(), 2));
}