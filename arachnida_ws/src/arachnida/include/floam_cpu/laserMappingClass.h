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
#include <pcl_ros/impl/transforms.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <math.h>
#include <vector>

#define LASER_CELL_WIDTH 50.0
#define LASER_CELL_HEIGHT 50.0
#define LASER_CELL_DEPTH 50.0

#define LASER_CELL_RANGE_HORIZONTAL 2
#define LASER_CELL_RANGE_VERTICAL 2

class LaserMappingClass {
    private:
        int originInMapX;
        int originInMapY;
        int originInMapZ;
        int mapWidth;
        int mapHeight;
        int mapDepth;
        std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>> map;
        pcl::VoxelGrid<pcl::PointXYZI> downSampledFilter;

        void addWidthCellNegative();
        void addWidthCellPositive();
        void addHeightCellNegative();
        void addHeightCellPositive();
        void addDepthCellNegative();
        void addDepthCellPositive();
        void checkPoints(int &x, int &y, int &z);
    
    public:
        LaserMappingClass();
        void init(double mapResolution);
        void updateCurrentPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, const Eigen::Isometry3d &poseCurrent);
        pcl::PointCloud<pcl::PointXYZI>::Ptr getMap();
};