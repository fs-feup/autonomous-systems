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
-----------------------------------------------------------------------------------------------
CalculateLongitudinalForces - test for Max torque , Medium Torque, Low torque , Negative torque
-----------------------------------------------------------------------------------------------
*/

TEST_F(VehicleModelTest, CalculateLongitudinalHigh){
  double Fx_FL = 0, Fx_FR = 0, Fx_RL = 0, Fx_RR = 0;
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = 60;
  newTorques.RR = 60;
  newTorques.timestamp = 0;
  vehicleModel.setTorques(newTorques);
  vehicleModel.getLongitudinalForces(Fx_FL, Fx_FR, Fx_RL, Fx_RR);
  //ASSERT_EQ(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques), 1); // 0.87396
  ASSERT_EQ(Fx_FL, 0);
  ASSERT_EQ(Fx_FR, 0);
  ASSERT_NEAR(Fx_RL,1033.25 , 1);
  ASSERT_NEAR(Fx_RR, 1033.25, 1);
}

TEST_F(VehicleModelTest, CalculateLongitudinalMid){
  double Fx_FL = 0, Fx_FR = 0, Fx_RL = 0, Fx_RR = 0;
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = 30;
  newTorques.RR = 30;
  newTorques.timestamp = 0;
  vehicleModel.setTorques(newTorques);
  vehicleModel.getLongitudinalForces(Fx_FL, Fx_FR, Fx_RL, Fx_RR);
  //ASSERT_EQ(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques), 1); // 0.732
  ASSERT_EQ(Fx_FL, 0);
  ASSERT_EQ(Fx_FR, 0);
  ASSERT_NEAR(Fx_RL,433.71 , 1);
  ASSERT_NEAR(Fx_RR, 433.71, 1);
}

TEST_F(VehicleModelTest, CalculateLongitudinalLow){
  double Fx_FL = 0, Fx_FR = 0, Fx_RL = 0, Fx_RR = 0;
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = 10;
  newTorques.RR = 10;
  newTorques.timestamp = 0;
  vehicleModel.setTorques(newTorques);
  vehicleModel.getLongitudinalForces(Fx_FL, Fx_FR, Fx_RL, Fx_RR);
  //ASSERT_EQ(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques), 1); // 0.64
  ASSERT_EQ(Fx_FL, 0);
  ASSERT_EQ(Fx_FR, 0);
  ASSERT_NEAR(Fx_RL,127.10 , 1);
  ASSERT_NEAR(Fx_RR, 127.10, 1);
}

TEST_F(VehicleModelTest, CalculateLongitudinalNeg){
  double Fx_FL = 0, Fx_FR = 0, Fx_RL = 0, Fx_RR = 0;
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = -10;
  newTorques.RR = -10;
  newTorques.timestamp = 0;
  vehicleModel.setTorques(newTorques);
  vehicleModel.getLongitudinalForces(Fx_FL, Fx_FR, Fx_RL, Fx_RR);
  //ASSERT_EQ(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques), 1); // 0.64
  ASSERT_EQ(Fx_FL, 0);
  ASSERT_EQ(Fx_FR, 0);
  ASSERT_NEAR(Fx_RL,-127.10 , 1);
  ASSERT_NEAR(Fx_RR,-127.10, 1);
}

/*
---------------------------------------------------------------
calculateEfficency tests - test for Low , Mid , High , Negative
---------------------------------------------------------------
*/

TEST_F(VehicleModelTest , testEfficencyLowT){
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = 5;
  newTorques.RR = 5;
  newTorques.timestamp = 0;
  vehicleModel.setTorques(newTorques);
  ASSERT_NEAR(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques) , 0.617 , 0.01);
}

TEST_F(VehicleModelTest , testEfficencyMidT){
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = 30;
  newTorques.RR = 30;
  newTorques.timestamp = 0;
  vehicleModel.setTorques(newTorques);
  ASSERT_NEAR(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques) , 0.732 , 0.01);
}

TEST_F(VehicleModelTest , testEfficencyHighT){
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = 60;
  newTorques.RR = 60;
  newTorques.timestamp = 0;
  vehicleModel.setTorques(newTorques);
  ASSERT_NEAR(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques) , 0.879 , 0.01);
}

TEST_F(VehicleModelTest, testEfficiencyNegT){
  Wheels newTorques;
  newTorques.FL = 0;
  newTorques.FR = 0;
  newTorques.RL = -5;
  newTorques.RR = -5;
  newTorques.timestamp = 0;
  vehicleModel.setTorques(newTorques);
  ASSERT_NEAR(vehicleModel.getPowertrainModel().calculateEfficiency(newTorques) , 0.617 , 0.01);
}
/*
-----------------------------------------------------
calculateNormalForces - test for speed 0 and positive
-----------------------------------------------------
*/

TEST_F(VehicleModelTest, TestCalculateNormalFZero) {
  double Fz_Front = 0, Fz_Rear = 0;
  vehicleModel.getNormalForces(Fz_Front, Fz_Rear);
  ASSERT_NEAR(Fz_Front, 516.98 ,1);
  ASSERT_NEAR(Fz_Rear, 466.36 , 1);
}

TEST_F(VehicleModelTest, TestCalculateNormalFPositive) {
  double Fz_Front = 0, Fz_Rear = 0;
  vehicleModel.setVelocity(Eigen::Vector3d(5, 0, 0)); // Set a positive velocity of 5 m/s in the x direction
  vehicleModel.getNormalForces(Fz_Front, Fz_Rear);
  ASSERT_NEAR(Fz_Front, 519.1 ,1);
  ASSERT_NEAR(Fz_Rear, 469.3 , 1);
}

/*
----------------------------------------------------------
calculateSlipAngles - test for rest , straight and turning
----------------------------------------------------------
*/

TEST_F(VehicleModelTest, TestCalculateSlipAnglesRest) {
  double kappaFront = 0, kappaRear = 0;
  vehicleModel.setVelocity(Eigen::Vector3d(0, 0, 0)); // Set velocity to zero
  vehicleModel.getSlipAngles(kappaFront, kappaRear);
  ASSERT_EQ(kappaFront, 0);
  ASSERT_EQ(kappaRear, 0);
}

TEST_F(VehicleModelTest, TestCalculateSlipAnglesStraight) {
  double kappaFront = 0, kappaRear = 0;
  vehicleModel.setVelocity(Eigen::Vector3d(5, 0, 0)); // Set a positive velocity of 5 m/s in the x direction
  vehicleModel.getSlipAngles(kappaFront, kappaRear);
  ASSERT_EQ(kappaFront, 0);
  ASSERT_EQ(kappaRear, 0);
}

TEST_F(VehicleModelTest, TestCalculateSlipAnglesTurning) {
  double kappaFront = 0, kappaRear = 0;
  vehicleModel.setVelocity(Eigen::Vector3d(5, 0, 0)); // Set a positive velocity of 5 m/s in the x direction
  vehicleModel.setSteeringSetpointFront(0.1); // Set a small steering angle
  vehicleModel.getSlipAngles(kappaFront, kappaRear);
  ASSERT_NEAR(kappaFront, -0.1, 0.01); // Expect a small slip angle for the front wheel
  ASSERT_EQ(kappaRear, 0); // Rear wheels should not have slip angle in this model
}

/*
---------------------------------------------------------------------
calculateWheelTorques - test for positive, negative and zero throtle
---------------------------------------------------------------------
*/

TEST_F(VehicleModelTest, TestCalculateWheelTorquesPositiveThrottle) {
  Wheels wheelTorques;
  vehicleModel.getPowertrainModel().calculateWheelTorques(1.0 , wheelTorques);
  ASSERT_EQ(wheelTorques.FL, 0);
  ASSERT_EQ(wheelTorques.FR, 0);
  ASSERT_NEAR(wheelTorques.RL, 60.0, 1.0);
  ASSERT_NEAR(wheelTorques.RR, 60.0, 1.0);
}

TEST_F(VehicleModelTest, TestCalculateWheelTorquesNegativeThrottle) {
  Wheels wheelTorques;
  vehicleModel.getPowertrainModel().calculateWheelTorques(-1.0 , wheelTorques);
  ASSERT_EQ(wheelTorques.FL, 0);
  ASSERT_EQ(wheelTorques.FR, 0);
  ASSERT_NEAR(wheelTorques.RL, -60.0, 1.0);
  ASSERT_NEAR(wheelTorques.RR, -60.0, 1.0);
}

TEST_F(VehicleModelTest, TestCalculateWheelTorquesZeroThrottle) {
  Wheels wheelTorques;
  vehicleModel.getPowertrainModel().calculateWheelTorques(0.0 , wheelTorques);
  ASSERT_EQ(wheelTorques.FL, 0);
  ASSERT_EQ(wheelTorques.FR, 0);
  ASSERT_EQ(wheelTorques.RL, 0);
  ASSERT_EQ(wheelTorques.RR, 0);
}


/*
--------------------------------------------------------------------------------------------------------------
calculateCurrent - Tested for positive, negative and zero torque and positive and zero wheelspeed at 600 volts
--------------------------------------------------------------------------------------------------------------
*/

TEST_F(VehicleModelTest, TestCalculateCurrentPositiveTorque) {
  Wheels wheelTorques;
  Wheels wheelSpeeds;
  wheelTorques.FL = 0;
  wheelTorques.FR = 0;
  wheelTorques.RL = 60.0;
  wheelTorques.RR = 60.0;
  
  wheelSpeeds.FL = 10.0;
  wheelSpeeds.FR = 10.0;
  wheelSpeeds.RL = 10.0;
  wheelSpeeds.RR = 10.0;

  double voltage = 600.0; 
  double current = vehicleModel.getPowertrainModel().calculateCurrent(wheelTorques, wheelSpeeds, voltage);
  
  ASSERT_NEAR(current, 0.2396, 0.01);
}

TEST_F(VehicleModelTest, TestCalculateCurrentNegativeTorque) {
  Wheels wheelTorques;
  Wheels wheelSpeeds;
  wheelTorques.FL = 0;
  wheelTorques.FR = 0;
  wheelTorques.RL = -60.0;
  wheelTorques.RR = -60.0;

  wheelSpeeds.FL = 10.0;
  wheelSpeeds.FR = 10.0;
  wheelSpeeds.RL = 10.0;
  wheelSpeeds.RR = 10.0;

  double voltage = 600.0; 
  double current = vehicleModel.getPowertrainModel().calculateCurrent(wheelTorques, wheelSpeeds, voltage);
  
  ASSERT_NEAR(current, -0.2396, 0.01);
}

TEST_F(VehicleModelTest, TestCalculateCurrentZeroTorque) {
  Wheels wheelTorques;
  Wheels wheelSpeeds;
  wheelTorques.FL = 0;
  wheelTorques.FR = 0;
  wheelTorques.RL = 0.0;
  wheelTorques.RR = 0.0;

  wheelSpeeds.FL = 10.0;
  wheelSpeeds.FR = 10.0;
  wheelSpeeds.RL = 10.0;
  wheelSpeeds.RR = 10.0;

  double voltage = 600.0; 
  double current = vehicleModel.getPowertrainModel().calculateCurrent(wheelTorques, wheelSpeeds, voltage);
  
  ASSERT_EQ(current, 0);
}

TEST_F(VehicleModelTest, TestCalculateCurrentZeroWheelspeed) {
  Wheels wheelTorques;
  Wheels wheelSpeeds;
  wheelTorques.FL = 0;
  wheelTorques.FR = 0;
  wheelTorques.RL = 60.0;
  wheelTorques.RR = 60.0;

  wheelSpeeds.FL = 0.0;
  wheelSpeeds.FR = 0.0;
  wheelSpeeds.RL = 0.0;
  wheelSpeeds.RR = 0.0;

  double voltage = 600.0; 
  double current = vehicleModel.getPowertrainModel().calculateCurrent(wheelTorques, wheelSpeeds, voltage);
  
  ASSERT_EQ(current, 0);
}

/*
----------------------------------------------------------------------------
calulateSteeringAngles - test for positive, negative and zero steering input
----------------------------------------------------------------------------
*/

TEST_F(VehicleModelTest, TestCalculateSteeringAnglesPositive) {
  Wheels steeringAngles;
  vehicleModel.getSteeringModel().calculateSteeringAngles(1.0, steeringAngles);
  ASSERT_NEAR(steeringAngles.FL, 1.08, 0.01);  
  ASSERT_NEAR(steeringAngles.FR, 0.92, 0.01);
  ASSERT_EQ(steeringAngles.RL, 0.0);
  ASSERT_EQ(steeringAngles.RR, 0.0);
}

TEST_F(VehicleModelTest, TestCalculateSteeringAnglesNegative) {
  Wheels steeringAngles;
  vehicleModel.getSteeringModel().calculateSteeringAngles(-1.0, steeringAngles);
  ASSERT_NEAR(steeringAngles.FL, -0.92, 0.01);  
  ASSERT_NEAR(steeringAngles.FR, -1.08, 0.01);
  ASSERT_EQ(steeringAngles.RL, 0.0);
  ASSERT_EQ(steeringAngles.RR, 0.0);
}

TEST_F(VehicleModelTest, TestCalculateSteeringAnglesZero) {
  Wheels steeringAngles;
  vehicleModel.getSteeringModel().calculateSteeringAngles(0.0, steeringAngles);
  ASSERT_EQ(steeringAngles.FL, 0.0);
  ASSERT_EQ(steeringAngles.FR, 0.0);
  ASSERT_EQ(steeringAngles.RL, 0.0);
  ASSERT_EQ(steeringAngles.RR, 0.0);
}

/*
----------------------------------------------------------------------------------
calculateSteeringWheelAngle - test for positive, negative and zero steering angles
----------------------------------------------------------------------------------
*/

TEST_F(VehicleModelTest, TestCalculateSteeringWheelAnglePositive) {
  Wheels steeringAngles;
  steeringAngles.FL = 1.0;
  steeringAngles.FR = 0.5;
  steeringAngles.RL = 0.0;
  steeringAngles.RR = 0.0;

  double steeringWheelAngle = vehicleModel.getSteeringModel().calculateSteeringWheelAngle(steeringAngles);
  ASSERT_NEAR(steeringWheelAngle, 5.35, 0.1);  
}

TEST_F(VehicleModelTest, TestCalculateSteeringWheelAngleNegative) {
  Wheels steeringAngles;
  steeringAngles.FL = -1.0;
  steeringAngles.FR = -0.5;
  steeringAngles.RL = 0.0;
  steeringAngles.RR = 0.0;

  double steeringWheelAngle = vehicleModel.getSteeringModel().calculateSteeringWheelAngle(steeringAngles);
  ASSERT_NEAR(steeringWheelAngle, -6.32, 0.1);  
}

TEST_F(VehicleModelTest, TestCalculateSteeringWheelAngleZero) {
  Wheels steeringAngles;
  steeringAngles.FL = 0.0;
  steeringAngles.FR = 0.0;
  steeringAngles.RL = 0.0;
  steeringAngles.RR = 0.0;

  double steeringWheelAngle = vehicleModel.getSteeringModel().calculateSteeringWheelAngle(steeringAngles);
  ASSERT_EQ(steeringWheelAngle, 0.0);
}

/*
-------------------------------------------------------------------------------------
calculateWheelGeometry - for positive negative and zero steering angles and velocities
-------------------------------------------------------------------------------------
*/

// positive steering and zero velocity
TEST_F(VehicleModelTest, TestCalculateWheelGeometryPositiveSteering) {
  double steeringFront;
  Eigen::Vector3d rFL, rFR, rRL, rRR, rFront, rRear, vFL, vFR, vRL, vRR;
  
  vehicleModel.setSteeringSetpointFront(0.1); // Set a small positive steering angle
  vehicleModel.getWheelGeometry(steeringFront, rFL, rFR, rRL, rRR, rFront, rRear, vFL, vFR, vRL, vRR);
  
  ASSERT_NEAR(steeringFront, 0.1, 0.01);
  ASSERT_NEAR(rFL.x(), 0.726, 0.01);
  ASSERT_NEAR(rFR.x(), 0.726, 0.01);
  ASSERT_NEAR(rRL.x(), -0.804, 0.01);
  ASSERT_NEAR(rRR.x(), -0.804, 0.01);
  ASSERT_NEAR(rFL.y(), 0.6, 0.01);
  ASSERT_NEAR(rFR.y(), -0.6, 0.01);
  ASSERT_NEAR(rRL.y(), 0.6, 0.01);
  ASSERT_NEAR(rRR.y(), -0.6, 0.01);
  ASSERT_NEAR(rFL.z(), 0.0, 0.01);
  ASSERT_NEAR(rFR.z(), 0.0, 0.01);
  ASSERT_NEAR(rRL.z(), 0.0, 0.01);
  ASSERT_NEAR(rRR.z(), 0.0, 0.01);
  ASSERT_NEAR(vFL.x(), 0.0, 0.01);
  ASSERT_NEAR(vFR.x(), 0.0, 0.01);
  ASSERT_NEAR(vRL.x(), 0.0, 0.01);
  ASSERT_NEAR(vRR.x(), 0.0, 0.01);
  ASSERT_NEAR(vFL.y(), 0.0, 0.01);
  ASSERT_NEAR(vFR.y(), 0.0, 0.01);
  ASSERT_NEAR(vRL.y(), 0.0, 0.01);
  ASSERT_NEAR(vRR.y(), 0.0, 0.01);
  ASSERT_NEAR(vFL.z(), 0.0, 0.01);
  ASSERT_NEAR(vFR.z(), 0.0, 0.01);
  ASSERT_NEAR(vRL.z(), 0.0, 0.01);
  ASSERT_NEAR(vRR.z(), 0.0, 0.01); 
}

// negative steering and zero velocity

TEST_F(VehicleModelTest, TestCalculateWheelGeometryNegativeSteering) {
  double steeringFront;
  Eigen::Vector3d rFL, rFR, rRL, rRR, rFront, rRear, vFL, vFR, vRL, vRR;
  
  vehicleModel.setSteeringSetpointFront(-0.1); // Set a small negative steering angle
  vehicleModel.getWheelGeometry(steeringFront, rFL, rFR, rRL, rRR, rFront, rRear, vFL, vFR, vRL, vRR);
  
  ASSERT_NEAR(steeringFront, -0.1, 0.01);
  ASSERT_NEAR(rFL.x(), 0.726, 0.01);
  ASSERT_NEAR(rFR.x(), 0.726, 0.01);
  ASSERT_NEAR(rRL.x(), -0.804, 0.01);
  ASSERT_NEAR(rRR.x(), -0.804, 0.01);
  ASSERT_NEAR(rFL.y(), 0.6, 0.01);
  ASSERT_NEAR(rFR.y(), -0.6, 0.01);
  ASSERT_NEAR(rRL.y(), 0.6, 0.01);
  ASSERT_NEAR(rRR.y(), -0.6, 0.01);
  ASSERT_NEAR(rFL.z(), 0.0, 0.01);
  ASSERT_NEAR(rFR.z(), 0.0, 0.01);
  ASSERT_NEAR(rRL.z(), 0.0, 0.01);
  ASSERT_NEAR(rRR.z(), 0.0, 0.01);
  ASSERT_NEAR(vFL.x(), 0.0, 0.01);
  ASSERT_NEAR(vFR.x(), 0.0, 0.01);
  ASSERT_NEAR(vRL.x(), 0.0, 0.01);
  ASSERT_NEAR(vRR.x(), 0.0, 0.01);
  ASSERT_NEAR(vFL.y(), 0.0, 0.01);
  ASSERT_NEAR(vFR.y(), 0.0, 0.01);
  ASSERT_NEAR(vRL.y(), 0.0, 0.01);
  ASSERT_NEAR(vRR.y(), 0.0, 0.01);
  ASSERT_NEAR(vFL.z(), 0.0, 0.01);
  ASSERT_NEAR(vFR.z(), 0.0, 0.01);
  ASSERT_NEAR(vRL.z(), 0.0, 0.01);
  ASSERT_NEAR(vRR.z(), 0.0, 0.01);
}

TEST_F(VehicleModelTest, TestCalculateWheelGeometryZeroSteering) {
  double steeringFront;
  Eigen::Vector3d rFL, rFR, rRL, rRR, rFront, rRear, vFL, vFR, vRL, vRR;
  
  vehicleModel.setSteeringSetpointFront(0.0); // Set zero steering angle
  vehicleModel.getWheelGeometry(steeringFront, rFL, rFR, rRL, rRR, rFront, rRear, vFL, vFR, vRL, vRR);
  
  ASSERT_NEAR(steeringFront, 0.0, 0.01);
  ASSERT_NEAR(rFL.x(), 0.726, 0.01);
  ASSERT_NEAR(rFR.x(), 0.726, 0.01);
  ASSERT_NEAR(rRL.x(), -0.804, 0.01);
  ASSERT_NEAR(rRR.x(), -0.804, 0.01);
  ASSERT_NEAR(rFL.y(), 0.6, 0.01);
  ASSERT_NEAR(rFR.y(), -0.6, 0.01);
  ASSERT_NEAR(rRL.y(), 0.6, 0.01);
  ASSERT_NEAR(rRR.y(), -0.6, 0.01);
  ASSERT_NEAR(rFL.z(), 0.0, 0.01);
  ASSERT_NEAR(rFR.z(), 0.0, 0.01);
  ASSERT_NEAR(rRL.z(), 0.0, 0.01);
  ASSERT_NEAR(rRR.z(), 0.0, 0.01);
  
  // Velocities should be zero
  ASSERT_NEAR(vFL.x(), 0.0, 0.01);
  ASSERT_NEAR(vFR.x(), 0.0, 0.01);
  ASSERT_NEAR(vRL.x(), 0.0, 0.01);
  ASSERT_NEAR(vRR.x(), 0.0, 0.01);
  ASSERT_NEAR(vFL.y(), 0.0, 0.01);
  ASSERT_NEAR(vFR.y(), 0.0, 0.01);
  ASSERT_NEAR(vRL.y(), 0.0, 0.01);
  ASSERT_NEAR(vRR.y(), 0.0, 0.01);
  ASSERT_NEAR(vFL.z(), 0.0, 0.01);
  ASSERT_NEAR(vFR.z(), 0.0, 0.01);
  ASSERT_NEAR(vRL.z(), 0.0, 0.01);
  ASSERT_NEAR(vRR.z(), 0.0, 0.01);
}


TEST_F(VehicleModelTest, TestCalculateWheelGeometryPositiveVelocity) {
  double steeringFront;
  Eigen::Vector3d rFL, rFR, rRL, rRR, rFront, rRear, vFL, vFR, vRL, vRR;
  vehicleModel.setVelocity(Eigen::Vector3d(5, 0, 0)); // Set a positive velocity of 5 m/s in the x direction
  vehicleModel.getWheelGeometry(steeringFront, rFL, rFR, rRL, rRR, rFront, rRear, vFL, vFR, vRL, vRR);
  ASSERT_NEAR(steeringFront, 0.0, 0.01);
  ASSERT_NEAR(rFL.x(), 0.726, 0.01);
  ASSERT_NEAR(rFR.x(), 0.726, 0.01);
  ASSERT_NEAR(rRL.x(), -0.804, 0.01);
  ASSERT_NEAR(rRR.x(), -0.804, 0.01);
  ASSERT_NEAR(rFL.y(), 0.6, 0.01);
  ASSERT_NEAR(rFR.y(), -0.6, 0.01);
  ASSERT_NEAR(rRL.y(), 0.6, 0.01);
  ASSERT_NEAR(rRR.y(), -0.6, 0.01);
  ASSERT_NEAR(rFL.z(), 0.0, 0.01);
  ASSERT_NEAR(rFR.z(), 0.0, 0.01);
  ASSERT_NEAR(rRL.z(), 0.0, 0.01);
  ASSERT_NEAR(rRR.z(), 0.0, 0.01);
  ASSERT_NEAR(vFL.x(), 5.0, 0.01);
  ASSERT_NEAR(vFR.x(), 5.0, 0.01);
  ASSERT_NEAR(vRL.x(), 5.0, 0.01);
  ASSERT_NEAR(vRR.x(), 5.0, 0.01);
  ASSERT_NEAR(vFL.y(), 0.0, 0.01);
  ASSERT_NEAR(vFR.y(), 0.0, 0.01);
  ASSERT_NEAR(vRL.y(), 0.0, 0.01);
  ASSERT_NEAR(vRR.y(), 0.0, 0.01);
  ASSERT_NEAR(vFL.z(), 0.0, 0.01);
  ASSERT_NEAR(vFR.z(), 0.0, 0.01);
  ASSERT_NEAR(vRL.z(), 0.0, 0.01);
  ASSERT_NEAR(vRR.z(), 0.0, 0.01);
}

/*
-------------------------------------------------------------
updateWheelSpeeds - tested for wheel speeds at 0 and positive
-------------------------------------------------------------
*/


TEST_F(VehicleModelTest, TestUpdateWheelSpeeds) {
    Eigen::Vector3d vFL(3.0, 0, 0), vFR(3.0, 0, 0), vRL(3.0, 0, 0), vRR(3.0, 0, 0);
    vehicleModel.setWheelSpeedsTester(vFL, vFR, vRL, vRR);

    // Calculate expected RPM
    double rpm2ms = 0.203 * 2 * M_PI / 60.0;
    double expectedRPM = 3.0 / rpm2ms;

    ASSERT_NEAR(vehicleModel.getWheelspeeds().FL, expectedRPM, 0.01);
    ASSERT_NEAR(vehicleModel.getWheelspeeds().FR, expectedRPM, 0.01);
    ASSERT_NEAR(vehicleModel.getWheelspeeds().RL, expectedRPM, 0.01);
    ASSERT_NEAR(vehicleModel.getWheelspeeds().RR, expectedRPM, 0.01);
}

TEST_F(VehicleModelTest, TestUpdateWheelSpeedsZero) {
    Eigen::Vector3d vFL(0.0, 0, 0), vFR(0.0, 0, 0), vRL(0.0, 0, 0), vRR(0.0, 0, 0);
    vehicleModel.setWheelSpeedsTester(vFL, vFR, vRL, vRR);

    ASSERT_EQ(vehicleModel.getWheelspeeds().FL, 0);
    ASSERT_EQ(vehicleModel.getWheelspeeds().FR, 0);
    ASSERT_EQ(vehicleModel.getWheelspeeds().RL, 0);
    ASSERT_EQ(vehicleModel.getWheelspeeds().RR, 0);
}

/*
-------------------------------------------------------------------------------------------------
getWheelPositions - tested for wheel positions at zero orientation and at a certain angle(90 deg)
-------------------------------------------------------------------------------------------------
*/


TEST_F(VehicleModelTest, TestGetWheelPositionsZeroOrientation) {
    vehicleModel.setPosition(Eigen::Vector3d(0, 0, 0));
    vehicleModel.setOrientation(Eigen::Vector3d(0, 0, 0));

    auto wheels = vehicleModel.getWheelPositions();


    ASSERT_NEAR(wheels[0].x(), 0.726, 0.01);  // FL
    ASSERT_NEAR(wheels[0].y(), 0.6, 0.01);
    ASSERT_NEAR(wheels[1].x(), 0.726, 0.01);  // FR
    ASSERT_NEAR(wheels[1].y(), -0.6, 0.01);
    ASSERT_NEAR(wheels[2].x(), -0.804, 0.01); // RL
    ASSERT_NEAR(wheels[2].y(), 0.6, 0.01);
    ASSERT_NEAR(wheels[3].x(), -0.804, 0.01); // RR
    ASSERT_NEAR(wheels[3].y(), -0.6, 0.01);
}


TEST_F(VehicleModelTest, TestGetWheelPositionsAngleOrientation) {
    vehicleModel.setPosition(Eigen::Vector3d(0, 0, 0));
    vehicleModel.setOrientation(Eigen::Vector3d(0, 0,  - M_PI / 2)); // Rotate 90 degrees

    auto wheels = vehicleModel.getWheelPositions();

    ASSERT_NEAR(wheels[0].x(), -0.6, 0.01);  // FL
    ASSERT_NEAR(wheels[0].y(), 0.726, 0.01);
    ASSERT_NEAR(wheels[1].x(), 0.6, 0.01);   // FR
    ASSERT_NEAR(wheels[1].y(), 0.726, 0.01);
    ASSERT_NEAR(wheels[2].x(), -0.6, 0.01);  // RL
    ASSERT_NEAR(wheels[2].y(), -0.804, 0.01);
    ASSERT_NEAR(wheels[3].x(), 0.6, 0.01);   // RR
    ASSERT_NEAR(wheels[3].y(), -0.804, 0.01);
}

TEST_F(VehicleModelTest, TestGetWheelPositionsNegativeAngleOrientation) {
    vehicleModel.setPosition(Eigen::Vector3d(1, 1, 1));
    vehicleModel.setOrientation(Eigen::Vector3d(0, 0, 0)); // Rotate -90 degrees

    auto wheels = vehicleModel.getWheelPositions();

    ASSERT_NEAR(wheels[0].x(), 1.726, 0.01);  // FL
    ASSERT_NEAR(wheels[0].y(), 1.6, 0.01);
    ASSERT_NEAR(wheels[1].x(), 1.726, 0.01);  // FR
    ASSERT_NEAR(wheels[1].y(), -0.6 + 1, 0.01);
    ASSERT_NEAR(wheels[2].x(), -0.804 + 1 , 0.01); // RL
    ASSERT_NEAR(wheels[2].y(), 1.6, 0.01);
    ASSERT_NEAR(wheels[3].x(), -0.804 + 1, 0.01); // RR
    ASSERT_NEAR(wheels[3].y(), -0.6 + 1, 0.01);
}

/*
-------------------------------------------------------------------------------------------------------------
calculateAccelerations - zero , straigth acceleration , straight braking , turning , drag and friction , yaw
-------------------------------------------------------------------------------------------------------------
*/

TEST_F(VehicleModelTest , TestcalcAccelZero){
  Eigen::Vector3d friction =  Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d result = vehicleModel.getCalculateAccelerations(0,0,0,0,0,0,0,0,friction);
  ASSERT_EQ(result.x() , 0);
  ASSERT_EQ(result.y() , 0);
  ASSERT_EQ(result.z() , 0);
}

/*
! Friction is being added like simple acceleration this is not very realistic from a standstill point could be an error/bug in the sim
*/
TEST_F(VehicleModelTest , TestcalcAccelZeroWFriction){
  Eigen::Vector3d friction =  Eigen::Vector3d(10, 0, 0);
  Eigen::Vector3d result = vehicleModel.getCalculateAccelerations(0,0,0,0,0,0,0,0,friction);
  ASSERT_NEAR(result.x() , 0.0498 , 0.01);
  ASSERT_EQ(result.y() , 0);
  ASSERT_EQ(result.z() , 0);
}

TEST_F(VehicleModelTest , TestcalcAccelStraight){
  Eigen::Vector3d friction =  Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d result = vehicleModel.getCalculateAccelerations(0,0,0,5,5,0,0,0,friction);
  ASSERT_NEAR(result.x() , 0.0498 , 0.01);
  ASSERT_EQ(result.y() , 0);
  ASSERT_EQ(result.z() , 0);
}

TEST_F(VehicleModelTest , TestcalcAccelStraightBraking){
  Eigen::Vector3d friction =  Eigen::Vector3d(2, 0, 0);
  Eigen::Vector3d result = vehicleModel.getCalculateAccelerations(0,0,0,-5,-5,0,0,0,friction);
  ASSERT_NEAR(result.x() , -8/200.707 , 0.01);
  ASSERT_EQ(result.y() , 0);
  ASSERT_EQ(result.z() , 0);
}

TEST_F(VehicleModelTest , TestcalcAccelTurning){
  Eigen::Vector3d friction =  Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d result = vehicleModel.getCalculateAccelerations(0,0,0,5,5,0.1,0,0,friction);
  ASSERT_NEAR(result.x() , 0.0498 , 0.01);
  ASSERT_NEAR(result.y() , -0.005 , 0.01); // Small lateral acceleration due to turning
  ASSERT_NEAR(result.z() , 0 , 0.01); // Very small rotation (not noticable in this case)
}

/*
------------------------------------------------------
getDynamicStates - test for zero and positive velocity
------------------------------------------------------
*/

// !Currently the fuction is dependent on the processSlipAngleLat function (which is wrong) so every y value will be 0.067313 instead of zero
// !Same thing applies for the yaw which will always be the expect value + 0.00331369

TEST_F(VehicleModelTest, TestGetDynamicStatesPositive) {
  vehicleModel.setVelocity(Eigen::Vector3d(0, 0, 0)); // Set velocity to zero
  vehicleModel.setSteeringSetpointFront(0.0); // Set steering angle to zero

  Eigen::Vector3d result = vehicleModel.getGetDynamicStates(1);

  ASSERT_EQ(result.x(), 0);
  ASSERT_NEAR(result.y(), 0 , 0.001 );
  ASSERT_NEAR(result.z(), 0  , 0.001); 

}
/*
---------------------------------
calculateSlipAngles
-----------------------------
*/
// TEST_F(VehicleModelTest, testSlipAngle){
//   double kappaF , kappaR;
//   vehicleModel.getSlipAngles( kappaF , kappaR);
//   ASSERT_EQ(kappaF , 0);
//   ASSERT_EQ(kappaR , 0);
// }
TEST_F(VehicleModelTest, testSlipAngle){
  double kappaF , kappaR;
  vehicleModel.getSlipAngles( kappaF , kappaR);
  ASSERT_EQ(kappaF , 0);
  ASSERT_EQ(kappaR , 0);
}

TEST_F(VehicleModelTest , testSlipAngleFast){
  double kappaF, kappaR;
  vehicleModel.setVelocity(Eigen::Vector3d(5,0,0));
  vehicleModel.getSlipAngles(kappaF, kappaR);
  ASSERT_EQ(kappaF , 0);
  ASSERT_EQ(kappaR , 0);
}

TEST_F(VehicleModelTest , testSlipAngleFastTurn){
  double kappaF, kappaR;
  vehicleModel.setVelocity(Eigen::Vector3d(5,5,0));
  vehicleModel.getSlipAngles(kappaF, kappaR);
  ASSERT_NEAR(kappaF , 0.7853 , 0.01);
  ASSERT_NEAR(kappaR , 0.7853 , 0.01);
}
// TEST_F(VehicleModelTest, testSlipAngle){
//   double kappaF , kappaR;
//   vehicleModel.getSlipAngles( kappaF , kappaR);
//   ASSERT_EQ(kappaF , 0);
//   ASSERT_EQ(kappaR , 0);
// }


TEST_F(VehicleModelTest, TestGetDynamicStatesAccelerating) {
  vehicleModel.setVelocity(Eigen::Vector3d(0, 0, 0));
  vehicleModel.setSteeringSetpointFront(0.0);
  vehicleModel.setTorques(Wheels{0, 0, 60, 60}); // Set back wheel torques to maximum (full acceleration)

  Eigen::Vector3d result  = vehicleModel.getGetDynamicStates(1);

  ASSERT_NEAR(result.x(), 10.3 , 0.1);
  ASSERT_NEAR(result.y(), 0 , 0.001 );
  ASSERT_NEAR(result.z(), 0, 0.001); 
}

TEST_F(VehicleModelTest, TestGetDynamicStatesBraking) {
  vehicleModel.setVelocity(Eigen::Vector3d(0, 0, 0));
  vehicleModel.setSteeringSetpointFront(0);
  vehicleModel.setTorques(Wheels{0, 0, -60, -60}); // Set back wheel torques to maximum (full braking)
  Eigen::Vector3d result = vehicleModel.getGetDynamicStates(1);

  ASSERT_NEAR(result.x(), -10.3 , 0.1);
  ASSERT_NEAR(result.y(), 0 , 0.001 );
  ASSERT_NEAR(result.z(), 0, 0.001); 
}

  