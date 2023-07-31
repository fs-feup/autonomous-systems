
#include "kalman_filter/ekf.hpp"

#include <Eigen/Dense>
#include <iostream>

#include "loc_map/data_structures.hpp"

double ExtendedKalmanFilter::max_landmark_distance = 10.0;

bool ExtendedKalmanFilter::cone_match(const double x_from_state, const double y_from_state,
                                      const double x_from_perception,
                                      const double y_from_perception,
                                      const double distance_to_vehicle) {
  double delta_x = x_from_state - x_from_perception;
  double delta_y = y_from_state - y_from_perception;
  double delta = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));
  auto limit_function = [](double distance) {
    double curvature = 8.0;
    double initial_limit = 0.5;
    return pow(M_E, distance / curvature) - (1 - initial_limit);
  };
  return limit_function(distance_to_vehicle) > delta;
}

/*--------------------Constructors--------------------*/

ExtendedKalmanFilter::ExtendedKalmanFilter(const MotionModel& motion_model,
                                           const ObservationModel& observation_model)
    : X(Eigen::VectorXf::Zero(3)),
      P(Eigen::MatrixXf::Zero(3, 3)),
      _last_update(std::chrono::high_resolution_clock::now()),
      _motion_model(motion_model),
      _observation_model(observation_model),
      _fixed_map(false) {}

void ExtendedKalmanFilter::prediction_step(const MotionUpdate& motion_update) {
  std::chrono::time_point<std::chrono::high_resolution_clock> now =
      std::chrono::high_resolution_clock::now();
  double delta =
      std::chrono::duration_cast<std::chrono::microseconds>(now - this->_last_update).count();
  this->_last_motion_update = motion_update;
  Eigen::VectorXf tempX = this->X;
  this->X = this->_motion_model.predict_expected_state(tempX, motion_update, delta / 1000000);
  Eigen::MatrixXf G =
      this->_motion_model.get_motion_to_state_matrix(tempX, motion_update, delta / 1000000);
  Eigen::MatrixXf R = this->_motion_model.get_process_noise_covariance_matrix(
      this->X.size());  // Process Noise Matrix
  this->P = G * this->P * G.transpose() + R;
  this->_last_update = now;
}

ExtendedKalmanFilter::ExtendedKalmanFilter(const MotionModel& motion_model,
                                           const ObservationModel& observation_model,
                                           const Mission& mission)
    : _last_update(std::chrono::high_resolution_clock::now()),
      _motion_model(motion_model),
      _observation_model(observation_model) {
  switch (mission) {
    case Mission::skidpad: {
      this->_fixed_map = true;
      this->X = Eigen::VectorXf::Zero(5);
      this->X(0) = -15.0;
      this->X(1) = 0.0;
      this->X(2) = 0.0;
      this->X(3) = -1.637208342552185;
      this->X(4) = 14.400202751159668;
      this->_colors.push_back(colors::Color::blue);
      this->X(5) = -2.216218948364258;
      this->X(6) = 11.487205505371094;
      this->_colors.push_back(colors::Color::blue);
      this->X(7) = -3.867227792739868;
      this->X(8) = 9.018211364746094;
      this->_colors.push_back(colors::Color::blue);
      this->X(9) = -6.336233615875244;
      this->X(10) = 7.367220401763916;
      this->_colors.push_back(colors::Color::blue);
      this->X(11) = -9.250235557556152;
      this->X(12) = 6.788230895996094;
      this->_colors.push_back(colors::Color::blue);
      this->X(13) = -12.160233497619629;
      this->X(14) = 7.367241382598877;
      this->_colors.push_back(colors::Color::blue);
      this->X(15) = -14.630228042602539;
      this->X(16) = 9.018250465393066;
      this->_colors.push_back(colors::Color::blue);
      this->X(17) = -16.28021812438965;
      this->X(18) = 11.48725700378418;
      this->_colors.push_back(colors::Color::blue);
      this->X(19) = -16.86020851135254;
      this->X(20) = 14.40025806427002;
      this->_colors.push_back(colors::Color::blue);
      this->X(21) = -16.280197143554688;
      this->X(22) = 17.313356399536133;
      this->_colors.push_back(colors::Color::blue);
      this->X(23) = -14.630188941955566;
      this->X(24) = 19.783050537109375;
      this->_colors.push_back(colors::Color::blue);
      this->X(25) = -12.16018295288086;
      this->X(26) = 21.43324089050293;
      this->_colors.push_back(colors::Color::blue);
      this->X(27) = -9.2501802444458;
      this->X(28) = 22.012731552124023;
      this->_colors.push_back(colors::Color::blue);
      this->X(29) = -6.336182594299316;
      this->X(30) = 21.43321990966797;
      this->_colors.push_back(colors::Color::blue);
      this->X(31) = -3.8671889305114746;
      this->X(32) = 19.783010482788086;
      this->_colors.push_back(colors::Color::blue);
      this->X(33) = -2.2161977291107178;
      this->X(34) = 17.313304901123047;
      this->_colors.push_back(colors::Color::blue);
      this->X(35) = 1.7456188201904297;
      this->X(36) = 21.9042911529541;
      this->_colors.push_back(colors::Color::blue);
      this->X(37) = 5.1885271072387695;
      this->X(38) = 24.20477867126465;
      this->_colors.push_back(colors::Color::blue);
      this->X(39) = 9.24983024597168;
      this->X(40) = 25.012163162231445;
      this->_colors.push_back(colors::Color::blue);
      this->X(41) = 13.310827255249023;
      this->X(42) = 24.204750061035156;
      this->_colors.push_back(colors::Color::blue);
      this->X(43) = 16.753820419311523;
      this->X(44) = 21.90423583984375;
      this->_colors.push_back(colors::Color::blue);
      this->X(45) = 19.05380630493164;
      this->X(46) = 18.461328506469727;
      this->_colors.push_back(colors::Color::blue);
      this->X(47) = 19.861791610717773;
      this->X(48) = 14.400125503540039;
      this->_colors.push_back(colors::Color::blue);
      this->X(49) = 19.05377769470215;
      this->X(50) = 10.339128494262695;
      this->_colors.push_back(colors::Color::blue);
      this->X(51) = 16.753765106201172;
      this->X(52) = 6.89613676071167;
      this->_colors.push_back(colors::Color::blue);
      this->X(53) = 13.310755729675293;
      this->X(54) = 4.596149444580078;
      this->_colors.push_back(colors::Color::blue);
      this->X(55) = 9.24975299835205;
      this->X(56) = 3.790163993835449;
      this->_colors.push_back(colors::Color::blue);
      this->X(57) = 5.188456058502197;
      this->X(58) = 4.596179008483887;
      this->_colors.push_back(colors::Color::blue);
      this->X(59) = 1.7455644607543945;
      this->X(60) = 6.896191120147705;
      this->_colors.push_back(colors::Color::blue);
      this->X(61) = -1.637208342552185;
      this->X(62) = 14.400202751159668;
      this->_colors.push_back(colors::Color::blue);
      this->X(63) = 16.861791610717773;
      this->X(64) = 14.40013599395752;
      this->_colors.push_back(colors::Color::yellow);
      this->X(65) = 16.28278160095215;
      this->X(66) = 11.487138748168945;
      this->_colors.push_back(colors::Color::yellow);
      this->X(67) = 14.6317720413208;
      this->X(68) = 9.018143653869629;
      this->_colors.push_back(colors::Color::yellow);
      this->X(69) = 12.162766456604004;
      this->X(70) = 7.367153644561768;
      this->_colors.push_back(colors::Color::yellow);
      this->X(71) = 9.249764442443848;
      this->X(72) = 6.788164138793945;
      this->_colors.push_back(colors::Color::yellow);
      this->X(73) = 6.33656644821167;
      this->X(74) = 7.3671746253967285;
      this->_colors.push_back(colors::Color::yellow);
      this->X(75) = 3.8668723106384277;
      this->X(76) = 9.018182754516602;
      this->_colors.push_back(colors::Color::yellow);
      this->X(77) = 2.2166810035705566;
      this->X(78) = 11.487190246582031;
      this->_colors.push_back(colors::Color::yellow);
      this->X(79) = 1.637291669845581;
      this->X(80) = 14.400191307067871;
      this->_colors.push_back(colors::Color::yellow);
      this->X(81) = 2.2167022228240967;
      this->X(82) = 17.313289642333984;
      this->_colors.push_back(colors::Color::yellow);
      this->X(83) = 3.8669111728668213;
      this->X(84) = 19.782983779907227;
      this->_colors.push_back(colors::Color::yellow);
      this->X(85) = 6.336617469787598;
      this->X(86) = 21.43317413330078;
      this->_colors.push_back(colors::Color::yellow);
      this->X(87) = 9.2498197555542;
      this->X(88) = 22.012662887573242;
      this->_colors.push_back(colors::Color::yellow);
      this->X(89) = 12.162817001342773;
      this->X(90) = 21.43315315246582;
      this->_colors.push_back(colors::Color::yellow);
      this->X(91) = 14.631811141967773;
      this->X(92) = 19.782943725585938;
      this->_colors.push_back(colors::Color::yellow);
      this->X(93) = 16.28280258178711;
      this->X(94) = 17.3132381439209;
      this->_colors.push_back(colors::Color::yellow);
      this->X(95) = -1.7451812028884888;
      this->X(96) = 21.9043025970459;
      this->_colors.push_back(colors::Color::yellow);
      this->X(97) = -5.188172817230225;
      this->X(98) = 24.204816818237305;
      this->_colors.push_back(colors::Color::yellow);
      this->X(99) = -9.25016975402832;
      this->X(100) = 25.012231826782227;
      this->_colors.push_back(colors::Color::yellow);
      this->X(101) = -13.310173034667969;
      this->X(102) = 24.204845428466797;
      this->_colors.push_back(colors::Color::yellow);
      this->X(103) = -16.750181198120117;
      this->X(104) = 21.90435791015625;
      this->_colors.push_back(colors::Color::yellow);
      this->X(105) = -19.050193786621094;
      this->X(106) = 18.46146583557129;
      this->_colors.push_back(colors::Color::yellow);
      this->X(107) = -19.86020851135254;
      this->X(108) = 14.4002685546875;
      this->_colors.push_back(colors::Color::yellow);
      this->X(109) = -19.050222396850586;
      this->X(110) = 10.339265823364258;
      this->_colors.push_back(colors::Color::yellow);
      this->X(111) = -16.75023651123047;
      this->X(112) = 6.8962578773498535;
      this->_colors.push_back(colors::Color::yellow);
      this->X(113) = -13.3102445602417;
      this->X(114) = 4.600245475769043;
      this->_colors.push_back(colors::Color::yellow);
      this->X(115) = -9.25024700164795;
      this->X(116) = 3.7902307510375977;
      this->_colors.push_back(colors::Color::yellow);
      this->X(117) = -5.188243865966797;
      this->X(118) = 4.596216201782227;
      this->_colors.push_back(colors::Color::yellow);
      this->X(119) = -1.745235562324524;
      this->X(120) = 6.896203517913818;
      this->_colors.push_back(colors::Color::yellow);
      this->X(121) = 16.861791610717773;
      this->X(122) = 14.40013599395752;
      this->_colors.push_back(colors::Color::yellow);
      this->X(123) = -1.6372408866882324;
      this->X(124) = 5.400203227996826;
      this->_colors.push_back(colors::Color::orange);
      this->X(125) = -1.637248158454895;
      this->X(126) = 3.4002034664154053;
      this->_colors.push_back(colors::Color::orange);
      this->X(127) = 1.6372591257095337;
      this->X(128) = 5.400191783905029;
      this->_colors.push_back(colors::Color::orange);
      this->X(129) = 1.637251853942871;
      this->X(130) = 3.40019154548645;
      this->_colors.push_back(colors::Color::orange);
      this->X(131) = -1.6371756792068481;
      this->X(132) = 23.400203704833984;
      this->_colors.push_back(colors::Color::orange);
      this->X(133) = -1.637168526649475;
      this->X(134) = 25.400203704833984;
      this->_colors.push_back(colors::Color::orange);
      this->X(135) = -1.6371612548828125;
      this->X(136) = 27.400203704833984;
      this->_colors.push_back(colors::Color::orange);
      this->X(137) = -1.63715398311615;
      this->X(138) = 29.400203704833984;
      this->_colors.push_back(colors::Color::orange);
      this->X(139) = -1.6371467113494873;
      this->X(140) = 31.400203704833984;
      this->_colors.push_back(colors::Color::orange);
      this->X(141) = -1.6371395587921143;
      this->X(142) = 33.400203704833984;
      this->_colors.push_back(colors::Color::orange);
      this->X(143) = 1.637324333190918;
      this->X(144) = 23.400192260742188;
      this->_colors.push_back(colors::Color::orange);
      this->X(145) = 1.637331485748291;
      this->X(146) = 25.400192260742188;
      this->_colors.push_back(colors::Color::orange);
      this->X(147) = 1.6373387575149536;
      this->X(148) = 27.400192260742188;
      this->_colors.push_back(colors::Color::orange);
      this->X(149) = 1.6373460292816162;
      this->X(150) = 29.400192260742188;
      this->_colors.push_back(colors::Color::orange);
      this->X(151) = 1.6373533010482788;
      this->X(152) = 31.400192260742188;
      this->_colors.push_back(colors::Color::orange);
      this->X(153) = 1.6373604536056519;
      this->X(154) = 33.40018844604492;
      this->_colors.push_back(colors::Color::orange);
      this->X(155) = -1.6371322870254517;
      this->X(156) = 35.400203704833984;
      this->_colors.push_back(colors::Color::orange);
      this->X(157) = 1.6373677253723145;
      this->X(158) = 35.40018844604492;
      this->_colors.push_back(colors::Color::orange);
      this->X(159) = -0.5501322746276855;
      this->X(160) = 35.40019989013672;
      this->_colors.push_back(colors::Color::orange);
      this->X(161) = 0.5498676896095276;
      this->X(162) = 35.40019226074219;
      this->_colors.push_back(colors::Color::orange);
      this->X(163) = -1.7835502624511719;
      this->X(164) = 15.88513469696045;
      this->_colors.push_back(colors::Color::large_orange);
      this->X(165) = -1.7835609912872314;
      this->X(166) = 12.915034294128418;
      this->_colors.push_back(colors::Color::large_orange);
      this->X(167) = 1.7831497192382812;
      this->X(168) = 15.88512134552002;
      this->_colors.push_back(colors::Color::large_orange);
      this->X(169) = 1.7831389904022217;
      this->X(170) = 12.915020942687988;
      this->_colors.push_back(colors::Color::large_orange);
      this->P = Eigen::MatrixXf::Zero(170, 170);
      this->P(0, 0) = 1.1;
      this->P(1, 1) = 1.1;
      this->P(2, 2) = 1.1;
      break;
    }
    case Mission::acceleration: {
      this->_fixed_map = true;
      this->X = Eigen::VectorXf::Zero(3);
      this->P = Eigen::MatrixXf::Zero(3, 3);
      break;
    }
    default: {
      this->_fixed_map = false;
      std::cout << "AUTOCROSSSS" << std::endl;
      this->X = Eigen::VectorXf::Zero(3);
      this->P = Eigen::MatrixXf::Zero(3, 3);
      break;
    }
  }
}

/*-----------------------Algorithms-----------------------*/

void ExtendedKalmanFilter::correction_step(const ConeMap& perception_map) {
  for (auto cone : perception_map.map) {
    ObservationData observation_data = ObservationData(cone.first.x, cone.first.y, cone.second);
    int landmark_index = this->discovery(observation_data);
    if (landmark_index == -1) {  // Too far away landmark
      continue;
    }
    Eigen::MatrixXf H = this->_observation_model.get_state_to_observation_matrix(
        this->X, landmark_index, this->X.size());
    Eigen::MatrixXf Q = this->_observation_model
                            .get_observation_noise_covariance_matrix();  // Observation Noise Matrix
    Eigen::MatrixXf K = this->get_kalman_gain(H, this->P, Q);
    Eigen::Vector2f z_hat = this->_observation_model.observation_model(this->X, landmark_index);
    Eigen::Vector2f z = Eigen::Vector2f(observation_data.position.x, observation_data.position.y);
    this->X = this->X + K * (z - z_hat);
    this->P = (Eigen::MatrixXf::Identity(this->P.rows(), this->P.cols()) - K * H) * this->P;
  }
}

Eigen::MatrixXf ExtendedKalmanFilter::get_kalman_gain(const Eigen::MatrixXf& H,
                                                      const Eigen::MatrixXf& P,
                                                      const Eigen::MatrixXf& Q) {
  Eigen::MatrixXf S = H * P * H.transpose() + Q;
  Eigen::MatrixXf K = P * H.transpose() * S.inverse();
  return K;
}

int ExtendedKalmanFilter::discovery(const ObservationData& observation_data) {
  Eigen::Vector2f landmark_absolute =
      this->_observation_model.inverse_observation_model(this->X, observation_data);
  double distance =
      std::sqrt(pow(observation_data.position.x, 2) + pow(observation_data.position.y, 2));
  if (distance > ExtendedKalmanFilter::max_landmark_distance) {
    return -1;
  }
  double best_delta = 1000000000.0;
  int best_index = -1;
  for (int i = 3; i < this->X.size() - 1; i += 2) {
    double delta_x = this->X(i) - landmark_absolute(0);
    double delta_y = this->X(i + 1) - landmark_absolute(1);
    double delta = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));
    if (delta < best_delta && this->_colors[(i - 3) / 2] == observation_data.color) {
      best_index = i;
      best_delta = delta;
    }
  }
  if (best_index != -1) {
    double score =
        ExtendedKalmanFilter::cone_match(this->X(best_index), this->X(best_index + 1),
                                         landmark_absolute(0), landmark_absolute(1), distance);
    if (score > 0) {
      return best_index;
    }
  }
  // if (best_index != -1 || this->_fixed_map) {
  //   return best_index;
  // }
  // If not found, add to the map
  this->X.conservativeResizeLike(Eigen::VectorXf::Zero(this->X.size() + 2));
  this->X(this->X.size() - 2) = landmark_absolute(0);
  this->X(this->X.size() - 1) = landmark_absolute(1);
  this->P.conservativeResizeLike(Eigen::MatrixXf::Zero(this->P.rows() + 2, this->P.cols() + 2));
  this->_colors.push_back(observation_data.color);
  return this->X.size() - 2;
}

void ExtendedKalmanFilter::update(VehicleState* vehicle_state, ConeMap* track_map) {
  vehicle_state->pose = Pose(X(0), X(1), X(2));
  track_map->map.clear();
  for (int i = 3; i < this->X.size() - 1; i += 2) {
    track_map->map.insert(
        std::pair<Position, colors::Color>(Position(X(i), X(i + 1)), this->_colors[(i - 3) / 2]));
  }
}
