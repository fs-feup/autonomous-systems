struct DataAssociationParameters {
  double max_landmark_distance = 50.0;  // Distance from the car from which observations are ignored
  double association_gate =
      0.43;  // maximum allowed malahanobis distance between matched landmark and observation
  double new_landmark_confidence_gate =
      0.8;  // minimum confidence to consider a landmark new and not outlier
  double observation_x_noise = 0.1;
  double observation_y_noise = 0.1;

  DataAssociationParameters() = default;
};