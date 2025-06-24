#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common_lib/structures/position.hpp"

/**
 * @brief Represents a prelemenary cone with position, size, and confidence attributes.
 *
 * The PreCone class encapsulates the properties of a detected cone, including its position,
 * whether it is large, and the confidence of the detection.
 */

class PreCone {
public:
  /**
   * @brief Constructs a PreCone object.
   * @param x The x-coordinate of the cone's position.
   * @param y The y-coordinate of the cone's position.
   * @param is_large Indicates if the cone is large.
   * @param confidence The confidence score of the detection.
   */
  PreCone(double x, double y, bool is_large, double confidence);

  /**
   * @brief Gets the position of the cone.
   * @return The position as a common_lib::structures::Position object.
   */
  common_lib::structures::Position get_position() const;

  /**
   * @brief Gets the size of the precone.
   * @return True if large, False otherwise.
   */
  bool get_is_large() const;

  /**
   * @brief Gets the confidence score of the detection.
   * @return The confidence value.
   */
  double get_confidence() const;

  /**
   * @brief Equality operator.
   * @param other The other PreCone to compare with.
   * @return True if all attributes are equal, false otherwise.
   */
  bool operator==(const PreCone& other) const {
    return position_ == other.get_position() && confidence_ == other.get_confidence() &&
           is_large_ == other.get_is_large();
  }

private:
  common_lib::structures::Position position_;
  bool is_large_;
  double confidence_;
};
