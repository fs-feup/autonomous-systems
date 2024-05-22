#ifndef ICP_H
#define ICP_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

/**
 * @class ICP
 * @brief Class for performing Iterative Closest Point (ICP) registration.
 */
class ICP{
  private:
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> _icp_; /**< ICP object */

  public:
    /**
     * @brief Constructor for the ICP class.
     * @param targetFile Path to the target point cloud file.
     * @param maxCorrespondenceDistance Maximum correspondence distance for ICP.
     * @param maxIteration Maximum number of iterations for ICP.
     * @param transformationEpsilon Transformation epsilon for convergence criteria.
     * @param euclideanFitnessEpsilon Euclidean fitness epsilon for convergence criteria.
     */
    ICP(std::string target_file, double max_correspondence_distance, long max_iteration, 
        double transformation_epsilon, double euclidean_fitness_epsilon);

    /**
     * @brief Execute the ICP registration.
     * @param source Source point cloud for registration.
     * @param final_cloud Final registered point cloud.
     * @return Fitness score after registration.
     */
    double executeICP(pcl::PointCloud<pcl::PointXYZI>::Ptr source, 
                      pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud);
};

#endif // ICP_H
