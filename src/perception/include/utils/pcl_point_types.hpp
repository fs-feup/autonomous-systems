#pragma once

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/impl/correspondence_estimation.hpp>
#include <pcl/registration/impl/icp.hpp>
#include <pcl/registration/impl/registration.hpp>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>

struct PointXYZIR {
  PCL_ADD_POINT4D;  // x, y, z, padding
  float intensity = 1.0f;
  uint16_t ring = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointXYZIR() = default;
  PointXYZIR(float x_, float y_, float z_, float intensity_ = 1.0f, uint16_t ring_ = 0)
      : x(x_), y(y_), z(z_), intensity(intensity_), ring(ring_) {}
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(uint16_t, ring,
                                                                                     ring))

PCL_INSTANTIATE(PCLBase, (PointXYZIR));
PCL_INSTANTIATE(SACSegmentation, (PointXYZIR));
PCL_INSTANTIATE(ExtractIndices, (PointXYZIR));
PCL_INSTANTIATE(EuclideanClusterExtraction, (PointXYZIR));
PCL_INSTANTIATE(VoxelGrid, (PointXYZIR));
PCL_INSTANTIATE(KdTreeFLANN, (PointXYZIR));
PCL_INSTANTIATE(search::KdTree, (PointXYZIR));
PCL_INSTANTIATE(IterativeClosestPoint, (PointXYZIR));
PCL_INSTANTIATE(RandomSampleConsensus, (PointXYZIR));
PCL_INSTANTIATE(SampleConsensusModelPlane, (PointXYZIR));
PCL_INSTANTIATE(Registration, (PointXYZIR)(PointXYZIR)(float));
PCL_INSTANTIATE(registration::CorrespondenceEstimationBase, (PointXYZIR)(PointXYZIR)(float));
PCL_INSTANTIATE(registration::CorrespondenceEstimation, (PointXYZIR)(PointXYZIR)(float));

template class pcl::search::KdTree<PointXYZIR>;
template class pcl::KdTreeFLANN<PointXYZIR>;