#include <cone_evaluator/distance_predict.hpp>
#include <cmath>

#define CONE_WIDTH 0.228
#define CONE_HEIGHT 0.325


DistancePredict::DistancePredict(double vertical_ang_res, double horizontal_ang_res) :
            vertical_ang_res(vertical_ang_res), horizontal_ang_res(horizontal_ang_res) {}


double DistancePredict::evaluateCluster(Cluster& cluster) const {
    double coneDistance = cluster.getCentroid().norm();

    if (coneDistance <= 0) return 0;

    double vertical_component = CONE_HEIGHT / (2.0 * coneDistance *
                        tan(vertical_ang_res * M_PI / 180.0 / 2));
    double horizontal_component = CONE_WIDTH / (2.0 * coneDistance *
                        tan(horizontal_ang_res * M_PI / 180.0 / 2));

    double n_points_prediction = 0.5 * vertical_component * horizontal_component;

    double relative_error = abs(cluster.getPointCloud()->points.size() - n_points_prediction) /
                            n_points_prediction;

    return 1 - relative_error;
}