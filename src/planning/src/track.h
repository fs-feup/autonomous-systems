#include "vector"
#include "position.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>


using namespace std;

class Track{
    bool completed;
    vector<Position*> leftCones;
    vector<Position*> rightCones;

    public:
        Track();

        Position* getRightConeAt(int index);
        Position* getLeftConeAt(int index);

        int getRightConesSize();
        int getLeftConesSize();

        void fillTrack();

        void addConePair(Position* cone, string color); // adds the position of two cones to the track list

        void receiveConeData(); // TODO ros subscriber input (Position left, Position right, color);

        void sendTrack(); // TODO send track data to path planner

};
