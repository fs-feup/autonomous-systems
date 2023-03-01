#ifndef SRC_LOCALIZATION_MAPPING_EXPERIMENT1_INCLUDE_EXPERIMENT1_CAR_HPP_
#define SRC_LOCALIZATION_MAPPING_EXPERIMENT1_INCLUDE_EXPERIMENT1_CAR_HPP_

/**
 * @brief
 * Class for car
 */
class Car {
 protected:
 private:
  int speed;
  int weight;

 public:
  /// @brief
  /// @param speed
  /// @param weight
  Car(int speed, int weight);
  ~Car();

  void use_horn();
};

#endif  // SRC_LOCALIZATION_MAPPING_EXPERIMENT1_INCLUDE_EXPERIMENT1_CAR_HPP_