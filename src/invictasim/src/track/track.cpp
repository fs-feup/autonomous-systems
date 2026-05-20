#include "track/track.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

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
    common_lib::structures::Cone cone(x, y, color_str);
    const auto duplicate = std::find_if(cones_.begin(), cones_.end(),
                                        [&cone](const common_lib::structures::Cone& existing) {
                                          return existing.color == cone.color && existing == cone;
                                        });
    if (duplicate == cones_.end()) {
      cones_.push_back(cone);
    }
  }

  fit_start_line_to_track_width();
}

void Track::fit_start_line_to_track_width() {
  const double line_x = start_line_.second.x - start_line_.first.x;
  const double line_y = start_line_.second.y - start_line_.first.y;
  const double line_length = std::hypot(line_x, line_y);
  if (line_length <= std::numeric_limits<double>::epsilon() || cones_.size() < 2) {
    return;
  }

  const common_lib::structures::Position center(0.5 * (start_line_.first.x + start_line_.second.x),
                                                0.5 * (start_line_.first.y + start_line_.second.y));
  const double axis_x = line_x / line_length;
  const double axis_y = line_y / line_length;
  const double normal_x = -axis_y;
  const double normal_y = axis_x;

  struct Candidate {
    double along = 0.0;
    double score = std::numeric_limits<double>::max();
  };

  std::optional<Candidate> blue_candidate;
  std::optional<Candidate> yellow_candidate;
  std::optional<Candidate> best_any_candidate;
  std::optional<Candidate> second_any_candidate;

  auto consider_boundary = [](std::optional<Candidate>& candidate, double along, double score) {
    if (!candidate || score < candidate->score) {
      candidate = Candidate{along, score};
    }
  };

  for (const auto& cone : cones_) {
    const double rel_x = cone.position.x - center.x;
    const double rel_y = cone.position.y - center.y;
    const double along = rel_x * axis_x + rel_y * axis_y;
    const double plane_distance = std::abs(rel_x * normal_x + rel_y * normal_y);
    const double score = plane_distance + 0.25 * std::abs(along);

    if (cone.color == common_lib::competition_logic::Color::BLUE) {
      consider_boundary(blue_candidate, along, score);
    } else if (cone.color == common_lib::competition_logic::Color::YELLOW) {
      consider_boundary(yellow_candidate, along, score);
    }

    Candidate current{along, score};
    if (!best_any_candidate || score < best_any_candidate->score) {
      second_any_candidate = best_any_candidate;
      best_any_candidate = current;
    } else if ((!second_any_candidate || score < second_any_candidate->score) &&
               (!best_any_candidate || std::abs(along - best_any_candidate->along) >
                                           common_lib::structures::Cone::equality_tolerance)) {
      second_any_candidate = current;
    }
  }

  std::optional<double> lower_along;
  std::optional<double> upper_along;
  if (blue_candidate && yellow_candidate) {
    lower_along = std::min(blue_candidate->along, yellow_candidate->along);
    upper_along = std::max(blue_candidate->along, yellow_candidate->along);
  } else if (best_any_candidate && second_any_candidate) {
    lower_along = std::min(best_any_candidate->along, second_any_candidate->along);
    upper_along = std::max(best_any_candidate->along, second_any_candidate->along);
  }

  if (!lower_along || !upper_along ||
      (*upper_along - *lower_along) <= std::numeric_limits<double>::epsilon()) {
    return;
  }

  start_line_.first = common_lib::structures::Position(center.x + axis_x * *lower_along,
                                                       center.y + axis_y * *lower_along);
  start_line_.second = common_lib::structures::Position(center.x + axis_x * *upper_along,
                                                        center.y + axis_y * *upper_along);
}

const std::vector<common_lib::structures::Cone>& Track::get_cones() const { return cones_; }

common_lib::structures::Position Track::get_start_position() const { return start_position_; }

std::pair<common_lib::structures::Position, common_lib::structures::Position>
Track::get_start_line() const {
  return start_line_;
}
