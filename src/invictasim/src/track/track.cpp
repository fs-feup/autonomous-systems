#include "track/track.hpp"

Track::Track(std::string file_name) {
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("invictasim");
  std::string full_path = package_share_directory + "/resources/tracks/" + file_name + ".yaml";

  YAML::Node config = YAML::LoadFile(full_path);
  if (!config["track"]) {
    throw std::runtime_error("YAML missing 'track' root key in: " + full_path);
  }

  const YAML::Node& track_node = config["track"];
  if (track_node["start_position"]) {
    const YAML::Node& start_node = track_node["start_position"];
    if (!start_node.IsSequence() || start_node.size() < 2) {
      throw std::runtime_error("Malformed start_position. Expected [x, y] in: " + full_path);
    }
    start_position_ =
        common_lib::structures::Position(start_node[0].as<double>(), start_node[1].as<double>());
  }

  if (track_node["start_line"]) {
    const YAML::Node& start_line_node = track_node["start_line"];
    if (!start_line_node.IsSequence() || start_line_node.size() != 2 ||
        !start_line_node[0].IsSequence() || !start_line_node[1].IsSequence() ||
        start_line_node[0].size() < 2 || start_line_node[1].size() < 2) {
      throw std::runtime_error("Malformed start_line. Expected [[x1, y1], [x2, y2]] in: " +
                               full_path);
    }
    start_line_ = {common_lib::structures::Position(start_line_node[0][0].as<double>(),
                                                    start_line_node[0][1].as<double>()),
                   common_lib::structures::Position(start_line_node[1][0].as<double>(),
                                                    start_line_node[1][1].as<double>())};
  } else {
    start_line_ = {common_lib::structures::Position(start_position_.x, start_position_.y - 5.0),
                   common_lib::structures::Position(start_position_.x, start_position_.y + 5.0)};
  }

  if (!track_node["cones"]) {
    throw std::runtime_error("YAML missing 'cones' list in: " + full_path);
  }

  for (const auto& node : track_node["cones"]) {
    if (!node.IsSequence() || node.size() != 3) {
      throw std::runtime_error("Malformed cone entry. Expected [x, y, color] in: " + full_path);
    }

    double x = node[0].as<double>();
    double y = node[1].as<double>();
    std::string color_str = node[2].as<std::string>();
    this->cones_.emplace_back(x, y, color_str);
  }
  return;
}

const std::vector<common_lib::structures::Cone>& Track::get_cones() const { return cones_; }

common_lib::structures::Position Track::get_start_position() const { return start_position_; }

std::pair<common_lib::structures::Position, common_lib::structures::Position>
Track::get_start_line() const {
  return start_line_;
}
