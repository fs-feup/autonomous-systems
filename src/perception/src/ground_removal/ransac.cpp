#include "ground_removal/ransac.hpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

RANSAC::RANSAC(double epsilon, int n_tries) : epsilon(epsilon), n_tries(n_tries) {}


void RANSAC::groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, pcl::PointCloud<pcl::PointXYZI> &ret) const{

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_indices (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZI> seg;

    seg.setOptimizeCoefficients (true);

    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (epsilon);
    seg.setMaxIterations(n_tries);


    seg.setInputCloud (point_cloud);
    seg.segment (*inliers_indices, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(point_cloud);
    extract.setIndices(inliers_indices);

    extract.setNegative(true); // Extract outliers
    extract.filter(ret);
}