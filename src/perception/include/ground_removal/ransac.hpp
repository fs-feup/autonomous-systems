#include "ground_removal/ground_removal.hpp"
#include <pcl/sample_consensus/ransac.h>
#include <string>

/**
 * @class RANSAC
 * @brief Ground removal using the RANSAC algorithm.
 *
 * This class implements the GroundRemoval interface and performs ground removal
 * on a point cloud using the Random Sample Consensus (RANSAC) algorithm.
 */
class RANSAC : public GroundRemoval {
 private:
    double epsilon; ///< Epsilon threshold for ground removal.
    int n_tries;    ///< Number of RANSAC iterations.

 public:
    /**
     * @brief Constructor for the RANSAC ground removal algorithm.
     * @param epsilon Epsilon threshold for ground removal.
     * @param n_tries Number of RANSAC iterations.
     */
    RANSAC(double epsilon, int n_tries);

    /**
     * @brief Perform ground removal using the RANSAC algorithm.
     *
     * This function implements the ground removal using the RANSAC algorithm
     * on the provided point cloud.
     *
     * @param point_cloud The input point cloud to be processed.
     * @param[out] ret The resulting point cloud after ground removal.
     */
    void groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr ret) const override;
};