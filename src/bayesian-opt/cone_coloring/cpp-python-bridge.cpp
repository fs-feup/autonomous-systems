#include <cstdlib>  // For std::atof
#include <iostream>
#include <vector>

#include "../../src/planning/include/planning/cone_coloring.hpp"

Cone find_equivalent(Cone cone, const std::vector<Cone>& cones) {
  for (Cone c : cones) {
    if (c.position.euclidean_distance(cone.position) < 0.1) {
      return c;
    }
  }
  throw std::runtime_error("No equivalent cone found");
}

double evaluate_cone_coloring(const std::vector<Cone>& track,
                              std::pair<std::vector<Cone>, std::vector<Cone>> result) {
  double score = 0;
  // std::cout << "Left cones: ";
  for (Cone cone : result.first) {
    Cone match = find_equivalent(cone, track);
    // std::cout << "(" << cone.position.x << ", " << cone.position.y << "), ";
    if (match.color == Color::BLUE) {
      score++;
    } else {
      score--;
    }
  }
  // std::cout << std::endl << "Right cones: ";
  for (Cone cone : result.second) {
    Cone match = find_equivalent(cone, track);
    // std::cout << "(" << cone.position.x << ", " << cone.position.y << "), ";
    if (match.color == Color::YELLOW) {
      score++;
    } else {
      score--;
    }
  }
  // std::cout << std::endl;

  double final_score = score / static_cast<double>(track.size());
  double final_final_score = 100 * (1.0 - final_score);
  return final_final_score;
}

// Function to process the parameters (you can customize this as needed)
double evaluate_function(const ConeColoringConfig& params, const std::vector<Cone>& track) {
  ConeColoring cone_coloring(params);

  std::vector<Pose> poses = {
      Pose(-10, 10.8, 0),       Pose(2, 11, 0.26),      Pose(11.5, 13.5, 0.52), Pose(14.2, 14.2, 0),
      Pose(15.7, 13.6, 6),      Pose(18.5, 10, 4.71),   Pose(17.5, 2, 4.45),    Pose(-7, -14, 3.93),
      Pose(-12.3, -14.5, 2.62), Pose(-15, -11.5, 2.36), Pose(-16.5, -9, 1.57),  Pose(-15, 7, 1.57),
      Pose(-13.5, 9, 0.785)};

  double score = 0;
  for (const auto& pose : poses) {
    // std::cout << "\n\n\n NEW SCENARIO \n\n\n" << std::endl;
    std::pair<std::vector<Cone>, std::vector<Cone>> colored_cones =
        cone_coloring.color_cones(track, pose);
    // std::cout << "Number of output cones: "
    //           << colored_cones.first.size() + colored_cones.second.size() << std::endl;
    score += evaluate_cone_coloring(track, colored_cones);
  }

  return score;
}

int main(int argc, char* argv[]) {
  // Ensure we have the correct number of arguments
  if (argc != 7) {  // 6 parameters + program name
    std::cerr << "Usage: ./my_cpp_executable" << std::endl;
    return 1;
  }

  std::vector<Cone> track = {
      Cone(-16.7219, 9.56934, "blue_cone"),    Cone(-12.7553, 12.49, "blue_cone"),
      Cone(-6.89062, 13.0757, "blue_cone"),    Cone(9.62956, 15.8379, "blue_cone"),
      Cone(12.492, 17.0404, "blue_cone"),      Cone(15.1389, 16.5025, "blue_cone"),
      Cone(19.5034, 13.6361, "blue_cone"),     Cone(20.8922, 11.7012, "blue_cone"),
      Cone(17.5221, 15.3812, "blue_cone"),     Cone(21.4761, 8.79574, "blue_cone"),
      Cone(20.9992, 5.27662, "blue_cone"),     Cone(19.9925, 2.24053, "blue_cone"),
      Cone(19.0983, 0.0, "blue_cone"),         Cone(17.1824, -3.23994, "blue_cone"),
      Cone(11.114, -6.74408, "blue_cone"),     Cone(-4.06552, 13.3637, "blue_cone"),
      Cone(14.2831, -5.26437, "blue_cone"),    Cone(8.25363, -8.53889, "blue_cone"),
      Cone(5.06185, -10.1551, "blue_cone"),    Cone(1.42086, -11.9634, "blue_cone"),
      Cone(-2.4975, -14.0305, "blue_cone"),    Cone(-5.74864, -16.1217, "blue_cone"),
      Cone(-9.34841, -17.1551, "blue_cone"),   Cone(-12.2114, -16.6459, "blue_cone"),
      Cone(-14.4625, -14.9249, "blue_cone"),   Cone(-16.2427, -13.276, "blue_cone"),
      Cone(-0.131239, 13.3125, "blue_cone"),   Cone(-18.1431, -11.086, "blue_cone"),
      Cone(-18.6174, -7.56085, "blue_cone"),   Cone(-18.9382, -5.15509, "blue_cone"),
      Cone(-18.5558, -2.56017, "blue_cone"),   Cone(-18.1206, 0.0, "blue_cone"),
      Cone(-17.7841, 3.04246, "blue_cone"),    Cone(-17.8432, 6.27091, "blue_cone"),
      Cone(-15.1864, 11.7137, "blue_cone"),    Cone(3.50416, 13.7245, "blue_cone"),
      Cone(7.13676, 14.727, "blue_cone"),      Cone(-12.2184, 7.60803, "yellow_cone"),
      Cone(7.14787, 9.92656, "yellow_cone"),   Cone(10.4312, 10.799, "yellow_cone"),
      Cone(12.9655, 11.8014, "yellow_cone"),   Cone(14.9652, 11.2833, "yellow_cone"),
      Cone(16.6054, 9.12035, "yellow_cone"),   Cone(-6.90998, 8.44454, "yellow_cone"),
      Cone(16.7063, 6.10772, "yellow_cone"),   Cone(15.876, 3.47906, "yellow_cone"),
      Cone(15.106, 1.5027, "yellow_cone"),     Cone(13.6765, 0.0, "yellow_cone"),
      Cone(9.41953, -2.84739, "yellow_cone"),  Cone(12.0732, -1.43122, "yellow_cone"),
      Cone(7.26282, -3.9408, "yellow_cone"),   Cone(4.65159, -5.15509, "yellow_cone"),
      Cone(1.78774, -6.66723, "yellow_cone"),  Cone(-1.97969, -8.56329, "yellow_cone"),
      Cone(-4.01695, 8.50322, "yellow_cone"),  Cone(-5.18123, -10.5555, "yellow_cone"),
      Cone(-7.57043, -12.1125, "yellow_cone"), Cone(-9.76388, -12.7081, "yellow_cone"),
      Cone(-12.0338, -11.6718, "yellow_cone"), Cone(-13.9298, -8.98291, "yellow_cone"),
      Cone(-14.1575, -5.77329, "yellow_cone"), Cone(-14.0043, -2.62998, "yellow_cone"),
      Cone(-13.6087, 0.0, "yellow_cone"),      Cone(-13.3478, 3.05712, "yellow_cone"),
      Cone(-0.059759, 8.37591, "yellow_cone"), Cone(-13.3455, 5.78808, "yellow_cone"),
      Cone(3.5151, 8.6968, "yellow_cone"),     Cone(-9.61314, 13.0, "blue_cone"),
      Cone(-9.99934, 12.989, "blue_cone"),     Cone(-9.62148, 8.39323, "yellow_cone"),
      Cone(-9.98667, 8.39348, "yellow_cone")};

  // Convert command-line arguments to doubles and store in the struct
  ConeColoringConfig params;
  params.angle_weight_ = std::atof(argv[1]);       // Convert first argument to double
  params.distance_weight_ = std::atof(argv[2]);    // Convert second argument to double
  params.ncones_weight_ = std::atof(argv[3]);      // Convert third argument to double
  params.angle_exponent_ = std::atof(argv[4]);     // Convert first argument to double
  params.distance_exponent_ = std::atof(argv[5]);  // Convert second argument to double
  params.max_cost_ = std::atof(argv[6]);           // Convert third argument to double

  // for (Cone cone : track) {
  //   std::cout << "(" << cone.position.x << ", " << cone.position.y << "),";
  // }
  // std::cout << "\n\n\n" << std::endl;

  // Now params contains the parameters as doubles
  // You can use this struct to run your logic, simulations, etc.

  // Evaluate the function with the given parameters
  double result = evaluate_function(params, track);

  // Output the result (this will be captured by the Python subprocess)
  std::cout << result << std::endl;

  return 0;
}
