#pragma once

#include <Eigen/Dense>
#include <functional>
#include <map>
#include <memory>

/**
 * @brief Data Association Method class,
 * used to match observations to landmarks in the map
 */
class DataAssociationModel {
  /**
   * @brief Check if the observed landmark is a valid match to the expected
   * landmark It depends on external parameters like the distance to the vehicle
   * @param x_from_state The x coordinate of the expected landmark
   * @param y_from_state The y coordinate of the expected landmark
   * @param x_from_perception The x coordinate of the observed landmark
   * @param y_from_perception The y coordinate of the observed landmark
   * @param distance_to_vehicle The distance between the vehicle and the
   * observed landmark
   * @return bool True if the observed landmark is a valid match to the expected
   * landmark
   */
  virtual bool valid_match(const float delta, const float distance_to_vehicle) const = 0;

  float max_landmark_distance; /**< Maximum deviation of the landmark position
                                from the expected position when the landmark is
                                perceived to be 1 meter away */
protected:
  float get_max_landmark_distance() const;

public:
  /**
   * @brief Match the observed landmark to a landmark in the map
   * @param observed_landmark_absolute The observed landmark position in the map
   * @param expected_state The expected state vector, containing the map
   * landmarks
   * @return int The index of the matched landmark in the map, -1 if invalid, -2
   * if not matched
   */
  virtual int match_cone(const Eigen::Vector2f& observed_landmark_absolute,
                         const Eigen::VectorXf& expected_state) const = 0;

  explicit DataAssociationModel(float max_landmark_distance);

  virtual ~DataAssociationModel() = default;
};

/**
 * @brief Simple (Dumb) Maximum Likelihood Method class,
 * attempts to match the observed landmark to the closest landmark in the map
 */
class SimpleMaximumLikelihood : public DataAssociationModel {
  bool valid_match(const float delta, const float distance_to_vehicle) const override;

public:
  int match_cone(const Eigen::Vector2f& observed_landmark_absolute,
                 const Eigen::VectorXf& expected_state) const override;

  explicit SimpleMaximumLikelihood(float max_landmark_distance);
};

const std::map<std::string,
               std::function<std::shared_ptr<DataAssociationModel>(float max_landmark_distance)>,
               std::less<>>
    data_association_model_constructors = {
        {"simple_ml", [](float max_landmark_distance) -> std::shared_ptr<DataAssociationModel> {
           return std::make_shared<SimpleMaximumLikelihood>(max_landmark_distance);
         }}};