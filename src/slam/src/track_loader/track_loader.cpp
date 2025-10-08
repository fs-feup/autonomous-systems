#include "track_loader/track_loader.hpp"

Node get_child_node(Node parentNode, std::string tag) {
  if (parentNode[tag]) {
    Node childNode = parentNode[tag];
    return childNode;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("slam"), ("Failed loading the map, no " + tag).c_str());
    return Node();
  }
}

void add_landmarks(Eigen::VectorXd& track, const Node& list) {
  int N = list.size();
  int track_size = track.size();
  track.conservativeResize(track_size + N * 2);
  for (const_iterator it = list.begin(); it != list.end(); ++it, track_size += 2) {
    const Node& position = *it;
    std::vector<double> vi = position["position"].as<std::vector<double>>();
    track.segment(track_size, 2) << vi[0], vi[1];
  }
}

void load_initial_state(std::string mapPath, Eigen::Vector3d& start_pose, Eigen::VectorXd& track) {
  Node map = LoadFile(mapPath);
  // loads the whole track node
  Node track_node = get_child_node(map, "track");

  // loads the cones
  Node left = get_child_node(track_node, "left");
  Node right = get_child_node(track_node, "right");
  Node unknown = get_child_node(track_node, "unknown");
  // TODO: Catch pose size != 3

  std::vector<double> state_pose_vector = track_node["start"]["pose"].as<std::vector<double>>();
  if (state_pose_vector.size() != 3) {
    RCLCPP_ERROR(rclcpp::get_logger("slam"), "Start pose vector size is not 3!");
    throw std::runtime_error("Start pose vector size is not 3!");
  }
  start_pose = Eigen::Vector3d(state_pose_vector.data());

  add_landmarks(track, left);
  add_landmarks(track, right);
  add_landmarks(track, unknown);
}

void load_acceleration_track(Eigen::Vector3d& start_pose, Eigen::VectorXd& track) {
  std::string package_prefix = ament_index_cpp::get_package_prefix("slam");
  std::string mapPath = package_prefix + "/../../src/slam/maps/acceleration.yaml";
  load_initial_state(mapPath, start_pose, track);
  transform_track(track, start_pose);
}

void load_trackdrive_track(Eigen::Vector3d& start_pose, Eigen::VectorXd& track) {
  std::string package_prefix = ament_index_cpp::get_package_prefix("slam");
  std::string mapPath = package_prefix + "/../../src/slam/maps/trackdrive.yaml";
  load_initial_state(mapPath, start_pose, track);
  transform_track(track, start_pose);
}

void load_skidpad_track(Eigen::Vector3d& start_pose, Eigen::VectorXd& track) {
  std::string package_prefix = ament_index_cpp::get_package_prefix("slam");
  std::string mapPath = package_prefix + "/../../src/slam/maps/skidpad.yaml";
  load_initial_state(mapPath, start_pose, track);
  transform_track(track, start_pose);
}

void transform_track(Eigen::VectorXd& track, Eigen::Vector3d& start_pose) {
  double theta = start_pose(2);
  Eigen::Vector2d translation = start_pose.head<2>();

  // Compute inverse rotation matrix (transpose of rotation matrix)
  Eigen::Matrix2d rotation_matrix;
  rotation_matrix << cos(theta), -sin(theta), sin(theta), cos(theta);
  Eigen::Matrix2d inverse_rotation = rotation_matrix.transpose();

  for (int i = 0; i < track.size(); i += 2) {
    Eigen::Vector2d point(track[i], track[i + 1]);

    // Translate to pose frame and rotate
    point = inverse_rotation * (point - translation);

    track[i] = point[0];
    track[i + 1] = point[1];
  }
  start_pose = Eigen::Vector3d(0, 0, 0);  // Reset start pose to origin after transformation
}
