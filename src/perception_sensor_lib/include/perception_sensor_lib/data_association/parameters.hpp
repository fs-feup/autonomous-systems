struct DataAssociationParameters {
  double max_landmark_distance = 50.0;
  double association_gate = 4.991;
  double new_landmark_confidence_gate = 0.8;
  double observation_x_noise = 0.1;
  double observation_y_noise = 0.1;

  DataAssociationParameters() = default;
};