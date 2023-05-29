#include "../include/planning/track.hpp"

Track::Track() { completed = false; }

Track::~Track() {
  int lSize = leftCones.size();
  int rSize = rightCones.size();

  for (int i = 0; i < lSize; i++)
    delete leftCones[i];

  for (int i = 0; i < rSize; i++)
    delete rightCones[i];
}

void Track::fillTrack(const string& path) {
  string x, y, color;
  ifstream trackFile;
  trackFile.open(path);

  while (trackFile >> x >> y >> color) {
    std::cout << x << " " << y << "\n";
    float xValue = stof(x);
    float yValue = stof(y);

    addCone(xValue, yValue, color);
  }
}

Cone* Track::getLeftConeAt(int index) { return leftCones[index]; }

Cone* Track::getRightConeAt(int index) { return rightCones[index]; }

int Track::getRightConesSize() { return rightCones.size(); }

int Track::getLeftConesSize() { return leftCones.size(); }

void Track::addCone(float xValue, float yValue, const string& color) {
  if (color == "b" || color == "or") {
    rightCones.push_back(new Cone(this->rightCount * 2, xValue, yValue));
    rightCount++;
  } else if (color == "y" || color == "ol") {
    leftCones.push_back(new Cone(this->leftCount * 2 + 1, xValue, yValue));
    leftCount++;
    }
  else
    cout << "Error adding cone\n";
}

void Track::setCone(Cone* cone) {
  switch(cone->getId() % 2) {
      // todo binary search
      // todo orange cones case
      case 0:
        for (size_t i = 0; i < leftCones.size(); i++) {
          if (leftCones[i]->getId() == cone->getId()) {
            leftCones[i]->setX(cone->getX());
            leftCones[i]->setY(cone->getY());
            return;
          }
        }
        leftCones.push_back(cone);
        break;
      case 1:
        for (size_t i = 0; i < rightCones.size(); i++) {
          if (rightCones[i]->getId() == cone->getId()) {
            rightCones[i]->setX(cone->getX());
            rightCones[i]->setY(cone->getY());
            return;
          }       
        }
        rightCones.push_back(cone);
  }
}

Cone* Track::findCone(float x, float y) {
  for(size_t i = 0; i < leftCones.size(); i++) {
    if (leftCones[i]->getX() == x && leftCones[i]->getY() == y)
      return leftCones[i];
  }

  for(size_t i = 0; i < rightCones.size(); i++) {
    if (rightCones[i]->getX() == x && rightCones[i]->getY() == y)
      return rightCones[i];
  }

  return nullptr;
}
