#pragma once
#include <cstddef>
#include <cstdint>

struct LidarPoint {
  size_t index;  // index of the point in the cloud

  // Constants for your LiDAR point layout
  static constexpr size_t POINT_STEP = 26;
  static constexpr size_t OFFSET_X = 0;
  static constexpr size_t OFFSET_Y = 4;
  static constexpr size_t OFFSET_Z = 8;
  static constexpr size_t OFFSET_INTENSITY = 12;
  static constexpr size_t OFFSET_RING = 16;
  static constexpr size_t OFFSET_TIMESTAMP = 18;

  // Constructor
  LidarPoint(size_t idx) : index(idx) {}

  // Return the index in the raw array for each field
  size_t x() const { return index * POINT_STEP + OFFSET_X; }
  size_t y() const { return index * POINT_STEP + OFFSET_Y; }
  size_t z() const { return index * POINT_STEP + OFFSET_Z; }
  size_t intensity() const { return index * POINT_STEP + OFFSET_INTENSITY; }
  size_t ring() const { return index * POINT_STEP + OFFSET_RING; }
  size_t timestamp() const { return index * POINT_STEP + OFFSET_TIMESTAMP; }
};
