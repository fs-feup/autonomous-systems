#include <string>

#include "gtest/gtest.h"
#include "vehicle_model/FSFEUP02.hpp"
/**
 * @brief Example test
 */
TEST(invictasim, example_test) {
  // Just to see if tests are running
  EXPECT_EQ(1, 1);
}

TEST(FSFEUP02Model_, initializeVehicleModel) {
  InvictaSimParameters params = InvictaSimParameters();
  FSFEUP02Model vehicle(params);
  EXPECT_EQ(vehicle.get_model_name(), "FSFEUP02Model");
}
