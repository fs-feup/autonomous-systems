#pragma once
#include <cstddef>
#include <cstdint>

/**
 * @brief Constants and helper functions for accessing LiDAR point cloud data
 */

constexpr size_t POINT_STEP = 26;
constexpr size_t OFFSET_X = 0;
constexpr size_t OFFSET_Y = 4;
constexpr size_t OFFSET_Z = 8;
constexpr size_t OFFSET_INTENSITY = 12;
constexpr size_t OFFSET_RING = 16;
constexpr size_t OFFSET_TIMESTAMP = 18;

constexpr inline size_t PointX(size_t idx) { return idx * ::POINT_STEP + ::OFFSET_X; }
constexpr inline size_t PointY(size_t idx) { return idx * ::POINT_STEP + ::OFFSET_Y; }
constexpr inline size_t PointZ(size_t idx) { return idx * ::POINT_STEP + ::OFFSET_Z; }
constexpr inline size_t PointIntensity(size_t idx) {
  return idx * ::POINT_STEP + ::OFFSET_INTENSITY;
}
constexpr inline size_t PointRing(size_t idx) { return idx * ::POINT_STEP + ::OFFSET_RING; }
constexpr inline size_t PointTimestamp(size_t idx) {
  return idx * ::POINT_STEP + ::OFFSET_TIMESTAMP;
}
