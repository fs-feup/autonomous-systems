
#include "track.h"

Track::Track(){
    completed = false;
}

void Track::fillTrack(const string &path) {
    string x, y, color;
    ifstream trackFile;
    trackFile.open(path);

    while (trackFile >> x >> y >> color) {
        cout << x;
        float xValue = stof(x);
        float yValue = stof(y);
        addConePair(new Position(xValue, yValue), color);
    }
}

Position* Track::getLeftConeAt(int index){
    return leftCones[index];
}

Position* Track::getRightConeAt(int index){
    return rightCones[index];
}

int Track::getRightConesSize(){
    return rightCones.size();
}

int Track::getLeftConesSize() {
    return leftCones.size();
}

void Track::addConePair(Position* cone, const string &color){
    if (color == "b" || color == "or")
        rightCones.push_back(cone);
    else if (color == "y" || color == "ol")
        leftCones.push_back(cone);
    else cout << "Error adding cone\n";

}