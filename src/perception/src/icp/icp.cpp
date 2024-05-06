#include "icp/icp.hpp"


ICP::ICP(std::string targetFile, double maxCorrespondenceDistance, long maxIteration, 
        double transformationEpsilon, double euclideanFitnessEpsilon){
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(targetFile, *target_cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
    }

    icp.setInputTarget(target_cloud);

    icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);

    icp.setMaximumIterations(maxIteration);

    icp.setTransformationEpsilon(transformationEpsilon);

    icp.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
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