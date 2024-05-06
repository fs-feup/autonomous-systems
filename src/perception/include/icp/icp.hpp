#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

class ICP{
  private:
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

  public:
    
    ICP(std::string targetFile, double maxCorrespondenceDistance, long maxIteration, 
        double transformationEpsilon, double euclideanFitnessEpsilon);

    double executeICP(pcl::PointCloud<pcl::PointXYZI>::Ptr source, 
                      pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud);
};