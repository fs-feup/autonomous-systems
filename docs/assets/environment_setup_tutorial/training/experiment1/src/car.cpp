#include <iostream>

#include "experiment1/car.hpp"

Car::Car(int speed, int weight)
{
    this->speed = speed;
    this->weight = weight;
}

Car::~Car()
{
}

void Car::use_horn()
{
    std::cout << "Beep" << std::endl;
}
