#include <cstdio>

#include "experiment1/car.hpp" 

int main(int argc, char **argv) {
    (void) argc;
    (void) argv;

    printf("hello world experiment1 package\n");

    int arr[10];

    arr[1] = 5;

    Car car1 = Car(10, 10);
    car1.use_horn();

    return 0;
}