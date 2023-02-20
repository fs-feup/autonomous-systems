#include <iostream>
#include "pathplanner.h"

int main() {

    Track* track = new Track();
    track->fillTrack();
    PathPlanner* pathplanner = new PathPlanner(track);
    pathplanner->middlePath();


    return 0;
}
