#include <cmath>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "motion_lib/inverter_model/ramped_inverter.hpp"
#include "sensors/wss.hpp"
#include "track/track.hpp"
#include "vehicle_model/FSFEUP02.hpp"

namespace {

/// Every track the package ships, so a malformed or renamed track file fails a test rather than
/// the simulator at startup.
const std::vector<std::string> kTrackNames = {
    "Acceleration", "EnduranceFSG25", "Europarque", "FEUP",    "FSE23",   "FSE24",
    "FSG23",        "FSG24",          "FSG25",      "FSS22V1", "FSS22V2", "Skidpad"};

}  // namespace

// ---------------------------------------------------------------------------------------------
// Track loading
// ---------------------------------------------------------------------------------------------

TEST(Track, everyShippedTrackLoads) {
  for (const auto& name : kTrackNames) {
    const Track track(name);
    EXPECT_GT(track.get_cones().size(), 0u) << name << " has no cones";
    EXPECT_FALSE(track.get_timing_lines().empty()) << name << " has no timing line";
  }
}

TEST(Track, missingTrackThrows) { EXPECT_ANY_THROW(Track("ThisTrackDoesNotExist")); }

TEST(Track, duplicateConesAreDropped) {
  // FSG24 is a closed loop, so no two cones of the same colour should sit on the same spot.
  const Track track("FSG24");
  const auto& cones = track.get_cones();
  for (std::size_t i = 0; i < cones.size(); ++i) {
    for (std::size_t j = i + 1; j < cones.size(); ++j) {
      const bool same_place = std::hypot(cones[i].position.x - cones[j].position.x,
                                         cones[i].position.y - cones[j].position.y) < 1e-6;
      EXPECT_FALSE(same_place && cones[i].color == cones[j].color)
          << "duplicate cone at index " << i << " and " << j;
    }
  }
}

TEST(Track, accelerationStartScalesWithCarLength) {
  const Track track("Acceleration");

  // start_gap_to_timing_line puts the car's nose a fixed distance behind the line, so a longer
  // car has to start further back. Without that, the 75 m run is timed differently per car.
  const auto start_short_car = track.get_start_position(1.0);
  const auto start_long_car = track.get_start_position(2.0);
  EXPECT_LT(start_long_car.x, start_short_car.x);
  EXPECT_NEAR(start_short_car.x - start_long_car.x, 1.0, 1e-9);

  // The nose itself always lands at the same place.
  EXPECT_NEAR(start_short_car.x + 1.0, start_long_car.x + 2.0, 1e-9);
}

TEST(Track, startGapIsIgnoredWhenNotConfigured) {
  // FSG24 does not set start_gap_to_timing_line, so the configured start_position stands.
  const Track track("FSG24");
  const auto plain = track.get_start_position();
  const auto with_offset = track.get_start_position(1.775);
  EXPECT_NEAR(plain.x, with_offset.x, 1e-12);
  EXPECT_NEAR(plain.y, with_offset.y, 1e-12);
}

// ---------------------------------------------------------------------------------------------
// Ramped inverter
// ---------------------------------------------------------------------------------------------

class RampedInverterTest : public ::testing::Test {
protected:
  void SetUp() override {
    car_parameters_ = std::make_shared<common_lib::car_parameters::CarParameters>(
        "invictasim/vehicle_models", "02");
    inverter_ = std::make_unique<RampedInverter>(*car_parameters_);
  }

  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;
  std::unique_ptr<RampedInverter> inverter_;
};

TEST_F(RampedInverterTest, startsAtZeroAndRampsTowardsTheRequest) {
  const double first = inverter_->calculate_inverter_throttle(1.0, 0.001);
  EXPECT_GT(first, 0.0);
  EXPECT_LT(first, 1.0) << "a slew-rate limited inverter must not jump to the request";

  const double second = inverter_->calculate_inverter_throttle(1.0, 0.001);
  EXPECT_GT(second, first);
}

TEST_F(RampedInverterTest, reachesTheRequestAndStopsThere) {
  for (int step = 0; step < 10000; ++step) {
    inverter_->calculate_inverter_throttle(1.0, 0.001);
  }
  EXPECT_NEAR(inverter_->calculate_inverter_throttle(1.0, 0.001), 1.0, 1e-9);
}

TEST_F(RampedInverterTest, rampRateIsIndependentOfStepSize) {
  // Ten steps of 1 ms must land where one step of 10 ms does, or the substepping in the vehicle
  // model would change the inverter's behaviour.
  for (int step = 0; step < 10; ++step) {
    inverter_->calculate_inverter_throttle(1.0, 0.001);
  }
  const double fine = inverter_->calculate_inverter_throttle(1.0, 0.0);

  RampedInverter coarse_inverter(*car_parameters_);
  const double coarse = coarse_inverter.calculate_inverter_throttle(1.0, 0.010);

  EXPECT_NEAR(fine, coarse, 1e-9);
}

TEST_F(RampedInverterTest, resetReturnsToZero) {
  inverter_->calculate_inverter_throttle(1.0, 0.1);
  inverter_->reset();
  const double after_reset = inverter_->calculate_inverter_throttle(0.0, 0.001);
  EXPECT_NEAR(after_reset, 0.0, 1e-12);
}

TEST_F(RampedInverterTest, clampsTheRequestToTheValidRange) {
  for (int step = 0; step < 10000; ++step) {
    inverter_->calculate_inverter_throttle(5.0, 0.001);
  }
  EXPECT_NEAR(inverter_->calculate_inverter_throttle(5.0, 0.001), 1.0, 1e-9);
}

// ---------------------------------------------------------------------------------------------
// Wheel speed sensor
// ---------------------------------------------------------------------------------------------

class WssTest : public ::testing::Test {
protected:
  void SetUp() override {
    config_path_ =
        common_lib::config_load::get_config_yaml_path("invictasim", "invictasim/sensors", "wss");
    wss_ = std::make_unique<WSS>(config_path_);
  }

  std::string config_path_;
  std::unique_ptr<WSS> wss_;
};

TEST_F(WssTest, stationaryWheelsReadExactlyZero) {
  // The cutoff is evaluated on the true wheel speed, not on the noisy reading, so a parked car
  // must report exactly zero rather than noise about zero.
  for (int sample = 0; sample < 200; ++sample) {
    const auto rpm = wss_->simulate_wheel_speeds({0.0, 0.0, 0.0, 0.0});
    EXPECT_DOUBLE_EQ(rpm.front_left, 0.0);
    EXPECT_DOUBLE_EQ(rpm.front_right, 0.0);
    EXPECT_DOUBLE_EQ(rpm.rear_left, 0.0);
    EXPECT_DOUBLE_EQ(rpm.rear_right, 0.0);
  }
}

TEST_F(WssTest, reportsRpmNotRadiansPerSecond) {
  // 100 rad/s is about 955 rpm. Averaging washes out the noise, outliers and dropouts.
  constexpr double kRadS = 100.0;
  constexpr double kExpectedRpm = kRadS * 60.0 / (2.0 * M_PI);

  double sum = 0.0;
  constexpr int kSamples = 2000;
  for (int sample = 0; sample < kSamples; ++sample) {
    sum += wss_->simulate_wheel_speeds({kRadS, kRadS, kRadS, kRadS}).front_left;
  }
  EXPECT_NEAR(sum / kSamples, kExpectedRpm, 0.1 * kExpectedRpm);
}

TEST_F(WssTest, tracksEachWheelIndependently) {
  // A mix-up between wheel indices and config keys would make the slowest wheel report the
  // fastest wheel's speed.
  const auto rpm = wss_->simulate_wheel_speeds({100.0, 200.0, 300.0, 400.0});
  EXPECT_LT(rpm.front_left, rpm.front_right);
  EXPECT_LT(rpm.front_right, rpm.rear_left);
  EXPECT_LT(rpm.rear_left, rpm.rear_right);
}

TEST_F(WssTest, dropoutStatusHasOneFlagPerWheel) {
  wss_->simulate_wheel_speeds({50.0, 50.0, 50.0, 50.0});
  EXPECT_EQ(wss_->get_wheel_dropout_status().size(), 4u);
}

// ---------------------------------------------------------------------------------------------
// Vehicle model
// ---------------------------------------------------------------------------------------------

class VehicleModelTest : public ::testing::Test {
protected:
  void SetUp() override {
    params_ = std::make_unique<InvictaSimParameters>();
    model_ = std::make_unique<FSFEUP02Model>(*params_);
  }

  /// Advance the model for a while at a fixed input.
  void drive(double seconds, const common_lib::structures::Wheels& throttle, double steering) {
    constexpr double kDt = 0.001;
    for (int step = 0; step < static_cast<int>(seconds / kDt); ++step) {
      model_->step(kDt, throttle, steering);
    }
  }

  bool state_is_finite() const {
    return std::isfinite(model_->get_position_x()) && std::isfinite(model_->get_position_y()) &&
           std::isfinite(model_->get_yaw()) && std::isfinite(model_->get_velocity_x()) &&
           std::isfinite(model_->get_velocity_y()) && std::isfinite(model_->get_yaw_rate());
  }

  std::unique_ptr<InvictaSimParameters> params_;
  std::unique_ptr<FSFEUP02Model> model_;
};

TEST_F(VehicleModelTest, reportsItsName) { EXPECT_EQ(model_->get_model_name(), "FSFEUP02Model"); }

TEST_F(VehicleModelTest, staysStillWithNoInput) {
  drive(1.0, {0.0, 0.0, 0.0, 0.0}, 0.0);
  EXPECT_NEAR(model_->get_velocity_x(), 0.0, 1e-6);
  EXPECT_NEAR(model_->get_velocity_y(), 0.0, 1e-6);
  EXPECT_NEAR(model_->get_yaw_rate(), 0.0, 1e-6);
}

TEST_F(VehicleModelTest, acceleratesForwardOnThrottle) {
  drive(2.0, {0.0, 0.0, 0.5, 0.5}, 0.0);
  EXPECT_GT(model_->get_velocity_x(), 0.5) << "the car did not pull away";
  EXPECT_GT(model_->get_position_x(), 0.0);
  EXPECT_TRUE(state_is_finite());
}

TEST_F(VehicleModelTest, autonomousModeSlowsOnRegenAlone) {
  // The car has no brake actuator, so under a controller a negative command is a regen request
  // and the friction brakes stay out of it. Regen still has to slow the car down.
  ASSERT_EQ(params_->control_mode, "autonomous") << "this test covers the autonomous plant";

  drive(3.0, {0.0, 0.0, 1.0, 1.0}, 0.0);
  const double cruising_speed = model_->get_velocity_x();
  ASSERT_GT(cruising_speed, 2.0) << "needs to be moving before a regen request means anything";

  // Long enough for the inverter to slew across the full command range and then actually pull.
  drive(1.5, {0.0, 0.0, -1.0, -1.0}, 0.0);
  EXPECT_LT(model_->get_velocity_x(), cruising_speed) << "regen did not slow the car";
}

TEST_F(VehicleModelTest, autonomousModeLeavesTheFrictionBrakesAlone) {
  // Regen is weaker than the friction brakes by design. If the brakes were ever engaged under a
  // controller, a full negative command would stop the car far harder than regen can.
  drive(3.0, {0.0, 0.0, 1.0, 1.0}, 0.0);
  const double cruising_speed = model_->get_velocity_x();
  ASSERT_GT(cruising_speed, 2.0);

  drive(0.2, {0.0, 0.0, -1.0, -1.0}, 0.0);
  const double deceleration = (cruising_speed - model_->get_velocity_x()) / 0.2;

  const double regen_torque_limit =
      params_->car_parameters->inverter_parameters->max_torque *
      params_->car_parameters->inverter_parameters->regen_torque_fraction;
  const double gear_ratio = params_->car_parameters->transmission_parameters->gear_ratio;
  const double wheel_radius = params_->car_parameters->tire_parameters->effective_tire_r;
  const double regen_deceleration_limit =
      regen_torque_limit * gear_ratio / (wheel_radius * params_->car_parameters->total_mass);

  // Generous headroom for aero and rolling drag, which also slow the car.
  EXPECT_LT(deceleration, 2.0 * regen_deceleration_limit)
      << "decelerating harder than regen alone can; the friction brakes appear to be engaged";
}

TEST_F(VehicleModelTest, manualModeBrakesToAStopWithoutReversing) {
  // In manual mode the driver's friction brakes exist, and their torque is signed by wheel
  // direction, so they bring the car to rest rather than pushing it backwards.
  params_->control_mode = "manual";
  auto manual_model = std::make_unique<FSFEUP02Model>(*params_);

  constexpr double kDt = 0.001;
  for (int step = 0; step < 3000; ++step) {
    manual_model->step(kDt, {0.0, 0.0, 1.0, 1.0}, 0.0);
  }
  const double cruising_speed = manual_model->get_velocity_x();
  ASSERT_GT(cruising_speed, 2.0);

  for (int step = 0; step < 5000; ++step) {
    manual_model->step(kDt, {0.0, 0.0, -1.0, -1.0}, 0.0);
  }
  EXPECT_LT(manual_model->get_velocity_x(), 0.5) << "the friction brakes did not stop the car";
  EXPECT_GT(manual_model->get_velocity_x(), -0.1) << "braking must not reverse the car";
}

TEST_F(VehicleModelTest, manualModeBrakesHarderThanAutonomousRegen) {
  // The whole point of the manual/autonomous split: the driver has more stopping power than a
  // controller, which only has regen.
  const auto speed_after_braking = [this](const std::string& mode) {
    params_->control_mode = mode;
    FSFEUP02Model model(*params_);
    constexpr double kDt = 0.001;
    for (int step = 0; step < 3000; ++step) {
      model.step(kDt, {0.0, 0.0, 1.0, 1.0}, 0.0);
    }
    for (int step = 0; step < 300; ++step) {
      model.step(kDt, {0.0, 0.0, -1.0, -1.0}, 0.0);
    }
    return model.get_velocity_x();
  };

  EXPECT_LT(speed_after_braking("manual"), speed_after_braking("autonomous"))
      << "manual braking was not stronger than autonomous regen";
}

TEST_F(VehicleModelTest, steeringTurnsTheCarTheWayItIsAsked) {
  drive(2.0, {0.0, 0.0, 0.4, 0.4}, 0.0);
  const double straight_yaw = model_->get_yaw();

  drive(2.0, {0.0, 0.0, 0.4, 0.4}, 0.2);  // positive steering is to the left
  EXPECT_GT(model_->get_yaw(), straight_yaw);
  EXPECT_GT(model_->get_yaw_rate(), 0.0);
}

TEST_F(VehicleModelTest, turnsSymmetricallyLeftAndRight) {
  drive(2.0, {0.0, 0.0, 0.4, 0.4}, 0.15);
  const double left_yaw_rate = model_->get_yaw_rate();

  model_->reset();
  drive(2.0, {0.0, 0.0, 0.4, 0.4}, -0.15);
  const double right_yaw_rate = model_->get_yaw_rate();

  ASSERT_GT(left_yaw_rate, 0.0);
  EXPECT_NEAR(left_yaw_rate, -right_yaw_rate, 0.1 * std::abs(left_yaw_rate));
}

TEST_F(VehicleModelTest, ebsStopsTheWheels) {
  drive(2.0, {0.0, 0.0, 0.6, 0.6}, 0.0);
  ASSERT_GT(model_->get_wheels_speed().rear_left, 1.0);

  model_->set_ebs(true);
  drive(0.1, {0.0, 0.0, 0.6, 0.6}, 0.0);
  EXPECT_NEAR(model_->get_wheels_speed().rear_left, 0.0, 1e-9);
  EXPECT_NEAR(model_->get_wheels_speed().front_right, 0.0, 1e-9);
}

TEST_F(VehicleModelTest, resetReturnsToRest) {
  drive(2.0, {0.0, 0.0, 0.8, 0.8}, 0.1);
  ASSERT_GT(model_->get_velocity_x(), 0.5);

  model_->reset();
  EXPECT_NEAR(model_->get_velocity_x(), 0.0, 1e-12);
  EXPECT_NEAR(model_->get_velocity_y(), 0.0, 1e-12);
  EXPECT_NEAR(model_->get_yaw(), 0.0, 1e-12);
  EXPECT_NEAR(model_->get_position_x(), 0.0, 1e-12);
  EXPECT_NEAR(model_->get_wheels_speed().rear_left, 0.0, 1e-12);
}

TEST_F(VehicleModelTest, resetIsRepeatable) {
  // Two identical runs either side of a reset must agree, or the powertrain state is leaking
  // across runs and the tuning results are not reproducible.
  drive(1.5, {0.0, 0.0, 0.7, 0.7}, 0.05);
  const double first_x = model_->get_position_x();
  const double first_vx = model_->get_velocity_x();

  model_->reset();
  drive(1.5, {0.0, 0.0, 0.7, 0.7}, 0.05);

  EXPECT_NEAR(model_->get_position_x(), first_x, 1e-9);
  EXPECT_NEAR(model_->get_velocity_x(), first_vx, 1e-9);
}

TEST_F(VehicleModelTest, survivesALongRunWithoutGoingNonFinite) {
  drive(20.0, {0.0, 0.0, 1.0, 1.0}, 0.1);
  EXPECT_TRUE(state_is_finite()) << "the state went NaN or infinite";
}

TEST_F(VehicleModelTest, ignoresNonPositiveTimeSteps) {
  drive(1.0, {0.0, 0.0, 0.5, 0.5}, 0.0);
  const double before = model_->get_velocity_x();

  model_->step(0.0, {0.0, 0.0, 1.0, 1.0}, 0.0);
  model_->step(-0.01, {0.0, 0.0, 1.0, 1.0}, 0.0);
  model_->step(std::nan(""), {0.0, 0.0, 1.0, 1.0}, 0.0);

  EXPECT_NEAR(model_->get_velocity_x(), before, 1e-12);
  EXPECT_TRUE(state_is_finite());
}

TEST_F(VehicleModelTest, publishesAccelerationUnfiltered) {
  // The published acceleration is the instantaneous specific force; the filtered copy that drives
  // the load transfer model lags it, so during a hard launch the two must differ.
  drive(0.2, {0.0, 0.0, 1.0, 1.0}, 0.0);
  EXPECT_GT(model_->get_acceleration_x(), model_->get_filtered_acceleration_x());
}
