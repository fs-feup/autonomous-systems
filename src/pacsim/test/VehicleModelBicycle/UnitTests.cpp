#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include "../../include/VehicleModel/VehicleModelBicycle.hpp"

class VehicleModelTest : public ::testing::Test {
protected:
  VehicleModelBicycle vehicleModel;  // Declare as a member
  void SetUp() override {
    vehicleModel = VehicleModelBicycle();  // Proper instantiation
    auto node = YAML::LoadFile("../../../../src/pacsim/config/vehicleModel.yaml");
    auto configVehicleModel = node["vehicle_model"];
    ConfigElement configVehicleModelElem(configVehicleModel);
    vehicleModel.readConfig(configVehicleModelElem);
  }

};

TEST_F(VehicleModelTest, TestCalculateForcesZero) {
  double velocityX = 0;
  double downforce = 0;
  double drag = 0;
  vehicleModel.getAeroModel().calculateForces(velocityX, downforce, drag);
  ASSERT_EQ(downforce, 0);
  ASSERT_EQ(drag, 0);
}

TEST_F(VehicleModelTest, TestCalculateForcesPositive) {
  double velocityX = 5;
  double downforce = 0;
  double drag = 0;
  vehicleModel.getAeroModel().calculateForces(velocityX, downforce, drag);
  ASSERT_NEAR(downforce , 6.1017 , 0.1);
  ASSERT_NEAR(drag , -5.17935 , 0.1);
}

TEST_F(VehicleModelTest, TestCalculateForcesNegative) {
  double velocityX = -5;
  double downforce = 0;
  double drag = 0;
  vehicleModel.getAeroModel().calculateForces(velocityX, downforce, drag);
  ASSERT_NEAR(downforce , 6.1017 , 0.1);
  ASSERT_NEAR(drag , 5.17935 , 0.1);
}

TEST_F(VehicleModelTest, TestBlueCone) {
  ASSERT_EQ(1, 1);

}

TEST_F(VehicleModelTest, TestUndefinedCone) {

  ASSERT_EQ(1, 1);
}
