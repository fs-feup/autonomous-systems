#include "vector"
#include "position.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>


using namespace std;
/**
* Track class. Contains the track information data and the cones position
*/

class Track{
    bool completed;
    vector<Position*> leftCones; /**<left side cone list*/
    vector<Position*> rightCones; /**<right side cone list*/

    public:
        Track();

        /**
         * Access right cone in the list in a specified index
        */
        Position* getRightConeAt(int index);  
        /**
         * Access left cone in the list in a specified index
        */
        Position* getLeftConeAt(int index);

        /**
         * Get right cones number
        */
        int getRightConesSize();
        /**
         * Get left cones number
        */
        int getLeftConesSize();

        /**
         * Read file to populate track
        */
        void fillTrack(string path);

        /**
         * Add cone to list
        */
        void addConePair(Position* cone, string color); // adds the position of two cones to the track list

        void receiveConeData(); // TODO ros subscriber input (Position left, Position right, color);

        void sendTrack(); // TODO send track data to path planner

};
