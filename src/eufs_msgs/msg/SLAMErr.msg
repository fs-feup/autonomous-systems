# Message for the error of our SLAM algorithm. All of them are based on euclidean distances, besides the map similarity,
# which can be interpreted as a percentage of how well our algorithm describes the map.

std_msgs/Header header

float64 x_err
float64 y_err
float64 z_err
float64 x_orient_err
float64 y_orient_err
float64 z_orient_err
float64 w_orient_err
float64 map_similarity
