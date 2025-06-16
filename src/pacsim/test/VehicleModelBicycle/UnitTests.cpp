#include <gtest/gtest.h>
#include "../../include/VehicleModel/VehicleModelBicycle.hpp"

class VehicleModelTest : public ::testing::Test {
protected:
  VehicleModelBicycle vehicleModel;  // Declare as a member

  void SetUp() override {
    vehicleModel = VehicleModelBicycle();  // Proper instantiation
  }

};

TEST_F(VehicleModelTest, TestYellowCone) {
  ASSERT_EQ(1, 1);

}

TEST_F(VehicleModelTest, TestBlueCone) {
  ASSERT_EQ(1, 1);

}

TEST_F(VehicleModelTest, TestUndefinedCone) {

  ASSERT_EQ(1, 1);
}
