#ifndef SRC_COMMON_LIB_INCLUDE_COMMON_LIB_STRUCTURES_TRACK_SIDE_HPP_
#define SRC_COMMON_LIB_INCLUDE_COMMON_LIB_STRUCTURES_TRACK_SIDE_HPP_
namespace common_lib::structures {
/**
 * @brief define enum for right and left sides of the track.
 * Coherent with previous implementation that matched [true]
 * to left and [false] to right
 *
 */
enum class TrackSide { RIGHT = 0, LEFT = 1 };
}  // namespace common_lib::structures
#endif