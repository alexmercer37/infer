#ifndef _PCLHEAD_H
#define _PCLHEAD_H

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>

// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/visualization/pcl_visualizer.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>

#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>

#endif