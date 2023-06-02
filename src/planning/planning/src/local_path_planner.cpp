#include "../include/planning/local_path_planner.hpp"

LocalPathPlanner::LocalPathPlanner():track() {}

vector<Position*> LocalPathPlanner::processNewArray(Track* cone_array) {
    vector<Position*> path;
    for (int i = 0; i < cone_array->getLeftConesSize(); i++)
        this->track.setCone(cone_array->getLeftConeAt(i));

    for (int i = 0; i < cone_array->getRightConesSize(); i++)
        this->track.setCone(cone_array->getRightConeAt(i));

    delete(cone_array);

    DT dt;

    for (int i = 0; i < track.getLeftConesSize(); i++) {
        Cone* lCone = track.getLeftConeAt(i);
        dt.insert(Point(lCone->getX(), lCone->getY()));
    }

    for (int i = 0; i < track.getRightConesSize(); i++) {
        Cone* rCone = track.getRightConeAt(i);
        dt.insert(Point(rCone->getX(), rCone->getY()));
    }

    for (DT::Finite_edges_iterator it = dt.finite_edges_begin();
        it != dt.finite_edges_end(); ++it) {
        float x1 = it->first->vertex((it->second + 1) % 3)->point().x();
        float y1 = it->first->vertex((it->second + 1) % 3)->point().y();
        float x2 = it->first->vertex((it->second + 2) % 3)->point().x();
        float y2 = it->first->vertex((it->second + 2) % 3)->point().y();

        Cone* cone1 = track.findCone(x1, y1);
        Cone* cone2 = track.findCone(x2, y2);

        if (cone1 != nullptr && cone2 != nullptr
            && cone1->getId() % 2 != cone2->getId() % 2) {
            float xDist = cone2->getX() - cone1->getX();
            float yDist = cone2->getY() - cone1->getY();

            Position* position = new Position(cone1->getX() + xDist / 2, cone1->getY() + yDist / 2);
            path.push_back(position);
        }
    }

    return path;
}