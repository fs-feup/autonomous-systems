#include "vector"
#include "track.h"

using namespace std;

class PathPlanner{

    Track* track;
    vector<Position*> finalPath;

    public:

    PathPlanner(Track *track);

    void writeFinalPath();

    void middlePath(); // TODO calculate middle path using track info

        // void rrt();
        // void astar();
        // other future algorithms

    
        void sendPath(); // TODO send path to controller
};