#pragma once

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace invictasim::config {

/**
 * @brief Read a required scalar, naming the file and key when it is missing.
 *
 * The raw yaml-cpp exception for a missing key says nothing about which key or which file, which
 * leaves the operator guessing after a typo.
 *
 * @tparam T Scalar type to read.
 * @param parent Parent node.
 * @param key Key to read.
 * @param context Human-readable description of where this key lives.
 * @return The parsed value.
 * @throws std::runtime_error When the key is absent or cannot be parsed as T.
 */
template <typename T>
T require(const YAML::Node& parent, const std::string& key, const std::string& context) {
  if (!parent || !parent[key]) {
    throw std::runtime_error("Missing required key '" + key + "' in " + context);
  }
  try {
    return parent[key].as<T>();
  } catch (const YAML::Exception& error) {
    throw std::runtime_error("Key '" + key + "' in " + context +
                             " has the wrong type: " + error.what());
  }
}

/**
 * @brief Track names the installed package ships, sorted alphabetically.
 *
 * Read from the resources/tracks directory rather than a hard-coded list, so it cannot drift.
 */
std::vector<std::string> available_tracks();

/**
 * @brief Check that a value is one of a set of known options.
 *
 * @param value Configured value.
 * @param options Accepted values.
 * @param setting_name Name of the setting, for the error message.
 * @throws std::runtime_error When the value is not in options, listing what is accepted.
 */
void require_one_of(const std::string& value, const std::vector<std::string>& options,
                    const std::string& setting_name);

/**
 * @brief Check that a track exists, listing the installed tracks when it does not.
 *
 * @param track_name Configured track name.
 * @throws std::runtime_error When no such track is installed.
 */
void require_track(const std::string& track_name);

/**
 * @brief Warn when the discipline and the track do not plausibly belong together.
 *
 * Mismatches are not fatal, because a user may deliberately drive any layout, but they silently
 * produce nonsense lap times, so they are worth saying out loud.
 *
 * @param discipline Configured discipline.
 * @param track_name Configured track name.
 */
void warn_on_discipline_track_mismatch(const std::string& discipline,
                                       const std::string& track_name);

}  // namespace invictasim::config
