#pragma once

#include <yaml-cpp/yaml.h>

#include <string>

namespace invictasim::resources {

/**
 * @brief Absolute path to a file under the installed invictasim resources/ directory.
 *
 * Resolves through the ament index, so the path is valid for an installed package rather than
 * only on the machine the package was built on.
 *
 * @param relative_path Path relative to resources/, e.g. "meshes/ground/config.yaml".
 * @return Absolute path, or an empty string when the package share directory cannot be found.
 */
std::string path(const std::string& relative_path);

/**
 * @brief Load a YAML file from the resources/ directory, tolerating absence.
 *
 * Returns an undefined node when the file is missing or malformed instead of throwing, so callers
 * can fall back to their defaults. Malformed files are reported on stderr.
 *
 * @param relative_path Path relative to resources/.
 * @return Parsed node, or an undefined node on failure.
 */
YAML::Node load(const std::string& relative_path);

/**
 * @brief Distance from the car origin to the foremost point of its hitboxes.
 *
 * Read from resources/meshes/car/<config>/config.yaml. Used both to time the car by its nose
 * rather than its origin and to place it behind a track's start line.
 *
 * @param car_parameters_config Car mesh folder name, e.g. "02".
 * @return Forward offset in metres, or 0.0 when no hitboxes are configured.
 */
double car_front_offset(const std::string& car_parameters_config);

}  // namespace invictasim::resources
