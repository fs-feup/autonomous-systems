#include "config/validate.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "config/resource_paths.hpp"

namespace invictasim::config {
namespace {

std::string join(const std::vector<std::string>& values) {
  std::ostringstream joined;
  for (std::size_t i = 0; i < values.size(); ++i) {
    joined << (i == 0 ? "" : ", ") << '"' << values[i] << '"';
  }
  return joined.str();
}

}  // namespace

std::vector<std::string> available_tracks() {
  std::vector<std::string> tracks;
  const std::string directory = resources::path("tracks");
  if (directory.empty()) {
    return tracks;
  }

  std::error_code error_code;
  for (const auto& entry : std::filesystem::directory_iterator(directory, error_code)) {
    if (entry.path().extension() == ".yaml") {
      tracks.push_back(entry.path().stem().string());
    }
  }
  std::sort(tracks.begin(), tracks.end());
  return tracks;
}

void require_one_of(const std::string& value, const std::vector<std::string>& options,
                    const std::string& setting_name) {
  if (std::find(options.begin(), options.end(), value) != options.end()) {
    return;
  }
  throw std::runtime_error(setting_name + " is \"" + value + "\", which is not a known option. " +
                           "Accepted values: " + join(options) + ".");
}

void require_track(const std::string& track_name) {
  const std::vector<std::string> tracks = available_tracks();
  if (std::find(tracks.begin(), tracks.end(), track_name) != tracks.end()) {
    return;
  }

  if (tracks.empty()) {
    throw std::runtime_error(
        "track_name is \"" + track_name +
        "\" but no tracks are installed. Check that the invictasim resources were installed.");
  }
  throw std::runtime_error(
      "track_name is \"" + track_name +
      "\", which is not an installed track. Available tracks: " + join(tracks) + ".");
}

void warn_on_discipline_track_mismatch(const std::string& discipline,
                                       const std::string& track_name) {
  // Only the two single-run disciplines have a layout that is unambiguously theirs; autocross and
  // trackdrive share every closed-loop track, so there is nothing to check for them.
  const bool track_is_acceleration = track_name == "Acceleration";
  const bool track_is_skidpad = track_name == "Skidpad";
  const bool discipline_is_acceleration = discipline == "acceleration";
  const bool discipline_is_skidpad = discipline == "skidpad";

  if (track_is_acceleration == discipline_is_acceleration &&
      track_is_skidpad == discipline_is_skidpad) {
    return;
  }

  std::cerr << "[invictasim] Warning: discipline \"" << discipline << "\" with track \""
            << track_name << "\". Lap timing follows the discipline, so lap times on this track "
            << "will not mean what you expect. Set them consistently in "
            << "config/global/global_config.yaml and config/invictasim/global.yaml." << std::endl;
}

}  // namespace invictasim::config
