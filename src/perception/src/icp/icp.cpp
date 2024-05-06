#include "icp/icp.hpp"


ICP::ICP(std::string target_file, double max_correspondence_distance, long max_iteration, 
        double transformation_epsilon, double euclidean_fitness_epsilon){
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(target_file, *target_cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
    }

    icp.setInputTarget(target_cloud);

    icp.setMaxCorrespondenceDistance(max_correspondence_distance);

    icp.setMaximumIterations(max_iteration);

    icp.setTransformationEpsilon(transformation_epsilon);

    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
}

double ICP::executeICP(pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud){

    icp.setInputSource(source_cloud);

    // Align the clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    icp.align(*aligned_cloud);

    if (icp.hasConverged()) {
        *final_cloud = *aligned_cloud;
        return icp.getFitnessScore();
    } else {
        return -1;
    }
}