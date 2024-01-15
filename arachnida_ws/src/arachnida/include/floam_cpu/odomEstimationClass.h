// This is an optimized implementation of the algorithm described in the following paper:
// 	J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
// 	Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// And this work is modified based on Advanced implementation of LOAM (A-LOAM) by 
// Tong Qin               qintonguav@gmail.com
// Shaozu Cao 		 saozu.cao@connect.ust.hk

// Author of FLOAM
// Wang Han 
// Nanyang Technological University, Singapore
// Email: wh200720041@gmail.com 
// Homepage: https://wanghan.pro

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
#pragma once

#include <string>
#include <math.h>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "lidarOptimisation.h"
#include <ros/ros.h>

class odomEstimationClass {
    private:
        double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
        Eigen::Map<Eigen::Quaterniond> qWCurrent = Eigen::Map<Eigen::Quaterniond>(parameters);
        Eigen::Map<Eigen::Vector3d> tWCurrent = Eigen::Map<Eigen::Vector3d>(parameters + 4);
        Eigen::Isometry3d lastOdom;

        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdTreeEdgeMap;
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdTreeSurfMap;

        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterEdge;
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;

        pcl::CropBox<pcl::PointXYZI> cropBoxFilter;

        int optimisationCount;

        void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, const pcl::PointCloud<pcl::PointXYZI>::Ptr &mapIn, ceres::Problem &problem, ceres::LossFunction *lossFunction);
        void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, const pcl::PointCloud<pcl::PointXYZI>::Ptr &mapIn, ceres::Problem &problem, ceres::LossFunction *lossFunction);
        void addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &downSampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &downSampledSurfCloud);
        void pointAssociateToMap(pcl::PointXYZI const *const pIn, pcl::PointXYZI *const pOut);
        void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edgePCIn, pcl::PointCloud<pcl::PointXYZI>::Ptr &edgePCOut, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surfPCIn, pcl::PointCloud<pcl::PointXYZI>::Ptr &surfPCOut);

    public:
        Eigen::Isometry3d odom;
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerMap;
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfMap;

        odomEstimationClass();
        void init(double mapResolution);
        void initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edgeIn, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surfIn);
        void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edgeIn, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surfIn);
        void getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr &laserCloudMap);

};