#include "planning/track.hpp"

#include <string>

#include "utils/color.hpp"

Track::Track() { completed = false; }

Track::~Track() {
  int lSize = leftCones.size();
  int rSize = rightCones.size();

  for (int i = 0; i < lSize; i++) delete leftCones[i];

  for (int i = 0; i < rSize; i++) delete rightCones[i];
}

void Track::fillTrack(const std::string& path) {
  std::string x, y, color;
  std::ifstream trackFile;

  trackFile.open(path);

  while (trackFile >> x >> y >> color) {
    float xValue = stof(x);
    float yValue = stof(y);

    addCone(xValue, yValue, color);
  }
}

Cone* Track::getLeftConeAt(int index) { return leftCones[index]; }

Cone* Track::getRightConeAt(int index) { return rightCones[index]; }

int Track::getRightConesSize() { return rightCones.size(); }

int Track::getLeftConesSize() { return leftCones.size(); }

void Track::addCone(float xValue, float yValue, const std::string& color) {
  if (color == colors::color_names[colors::blue]) {
    rightCones.push_back(new Cone(this->rightCount * 2, xValue, yValue));
    rightCount++;
  } else if (color == colors::color_names[colors::yellow]) {
    leftCones.push_back(new Cone(this->leftCount * 2 + 1, xValue, yValue));
    leftCount++;
  } else if (color != colors::color_names[colors::orange] &&
             color != colors::color_names[colors::large_orange]) {
    std::cout << "Invalid color: " << color << std::endl;
  }

}

void Track::setCone(Cone* cone) {
  switch (cone->getId() % 2) {
    // todo binary search
    case colors::blue:
      for (size_t i = 0; i < leftCones.size(); i++) {
        if (leftCones[i]->getId() == cone->getId()) {
          leftCones[i]->setX(cone->getX());
          leftCones[i]->setY(cone->getY());
          return;
        }
      }
      leftCones.push_back(cone);
      break;
    case colors::yellow:
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

Cone* Track::findCone(int id) {
  for (size_t i = 0; i < leftCones.size(); i++) {
    if (leftCones[i]->getId() == id) return leftCones[i];
  }

  for (size_t i = 0; i < rightCones.size(); i++) {
    if (rightCones[i]->getId() == id) return rightCones[i];
  }

  return nullptr;
}

Cone* Track::findCone(float x, float y) {
  for (size_t i = 0; i < leftCones.size(); i++) {
    if (leftCones[i]->getX() == x && leftCones[i]->getY() == y) return leftCones[i];
  }

  for (size_t i = 0; i < rightCones.size(); i++) {
    if (rightCones[i]->getX() == x && rightCones[i]->getY() == y) return rightCones[i];
  }

  return nullptr;
}

void Track::reset() {
  leftCones.clear();
  rightCones.clear();
}
