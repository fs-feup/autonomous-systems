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
/*
-----------------------------------------------------------
CalculateForces test - test for negative, zero and positive
-----------------------------------------------------------
*/


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

/*
-----------------------------------------------------------------------------
CalculateLongitudinalForces - test for Max torque , Medium Torque, Low torque
-----------------------------------------------------------------------------
*/

TEST_F(VehicleModelTest, CaulcateLongitudinalHigh){
  double Fx_FL = 0, Fx_FR = 0, Fx_RL = 0, Fx_RR = 0;
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = 60;
  newTorques.RR = 60;
  vehicleModel.setTorques(newTorques);
  vehicleModel.getLongitudinalForces(Fx_FL, Fx_FR, Fx_RL, Fx_RR);
  //ASSERT_EQ(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques), 1); // 0.87396
  ASSERT_EQ(Fx_FL, 0);
  ASSERT_EQ(Fx_FR, 0);
  ASSERT_NEAR(Fx_RL,1033.25 , 1);
  ASSERT_NEAR(Fx_RR, 1033.25, 1);
}

TEST_F(VehicleModelTest, CaulcateLongitudinalMid){
  double Fx_FL = 0, Fx_FR = 0, Fx_RL = 0, Fx_RR = 0;
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = 30;
  newTorques.RR = 30;
  vehicleModel.setTorques(newTorques);
  vehicleModel.getLongitudinalForces(Fx_FL, Fx_FR, Fx_RL, Fx_RR);
  //ASSERT_EQ(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques), 1); // 0.732
  ASSERT_EQ(Fx_FL, 0);
  ASSERT_EQ(Fx_FR, 0);
  ASSERT_NEAR(Fx_RL,433.71 , 1);
  ASSERT_NEAR(Fx_RR, 433.71, 1);
}

TEST_F(VehicleModelTest, CaulcateLongitudinalLow){
  double Fx_FL = 0, Fx_FR = 0, Fx_RL = 0, Fx_RR = 0;
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = 10;
  newTorques.RR = 10;
  vehicleModel.setTorques(newTorques);
  vehicleModel.getLongitudinalForces(Fx_FL, Fx_FR, Fx_RL, Fx_RR);
  //ASSERT_EQ(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques), 1); // 0.64
  ASSERT_EQ(Fx_FL, 0);
  ASSERT_EQ(Fx_FR, 0);
  ASSERT_NEAR(Fx_RL,127.10 , 1);
  ASSERT_NEAR(Fx_RR, 127.10, 1);
}

/*
------------------------
calculateEfficency tests - test for Low , Mid , High
------------------------
*/

TEST_F(VehicleModelTest , testEfficencyLowT){
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = 5;
  newTorques.RR = 5;
  vehicleModel.setTorques(newTorques);
  ASSERT_NEAR(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques) , 0.617 , 0.01);
}

TEST_F(VehicleModelTest , testEfficencyMidT){
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = 30;
  newTorques.RR = 30;
  vehicleModel.setTorques(newTorques);
  ASSERT_NEAR(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques) , 0.732 , 0.01);
}

TEST_F(VehicleModelTest , testEfficencyHighT){
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = 60;
  newTorques.RR = 60;
  vehicleModel.setTorques(newTorques);
  ASSERT_NEAR(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques) , 0.879 , 0.01);
}
