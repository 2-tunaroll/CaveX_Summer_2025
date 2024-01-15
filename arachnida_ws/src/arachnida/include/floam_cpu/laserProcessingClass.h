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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

class Double2d {
    public:
        int id;
        double value;
        Double2d(int idIn, double valueIn);
};

class PointsInfo {
    public:
        int layer;
        double time;
        PointsInfo(int layerIn, double timeIn);
};

class LaserProcessingClass {
    public:
        LaserProcessingClass();
        void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutEdges, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutSurfaces);
        void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, std::vector<Double2d> &cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutEdges, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutSurfaces);
};