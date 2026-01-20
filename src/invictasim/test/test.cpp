#include <string>

#include "../include/vehicle_model/FSFEUP02_model.hpp"
#include "gtest/gtest.h"
/**
 * @brief Example test
 */
TEST(invictasim, example_test) {
  // Just to see if tests are running
  EXPECT_EQ(1, 1);
}

TEST(FSFEUP02Model_, initializeVehicleModel) {
  std::string config_path = "/home/ws/config/invictasim/vehicle_models/FSFEUP02_model.yaml";
  FSFEUP02Model vehicle(config_path);
  EXPECT_EQ(vehicle.get_model_name(), "FSFEUP02Model");
}

TEST(TireModel_, initializeTireModel) {
  std::string config_path = "/home/ws/config/invictasim/vehicle_models/FSFEUP02_model.yaml";
  FSFEUP02Model vehicle(config_path);
  EXPECT_EQ(vehicle.get_tire_effective_radius(), 10);
}
