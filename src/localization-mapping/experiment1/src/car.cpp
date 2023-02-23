#include "experiment1/car.hpp"

#include <iostream>

Car::Car(int speed, int weight) {
  this->speed = speed;
  this->weight = weight;
}

Car::~Car() {}

void Car::use_horn() { std::cout << "Beep" << std::endl; }
