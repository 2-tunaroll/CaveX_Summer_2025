#pragma once

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>

#include "objBox.h"

class objPointCloudProcessor {
    public:
        objPointCloudProcessor();
        ~objPointCloudProcessor();
        
        void numPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
            const float filterRes, 
            Eigen::Vector4f minPoint, 
            Eigen::Vector4f maxPoint
        );

        std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> separateClouds(
            pcl::PointIndices::Ptr inliers, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud
        );

        std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentPlane(
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
            const int maxIterations, 
            const float distanceThreshold
        );

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterCloud(
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
            const float clusterTolerance, 
            const int minSize, 
            const int maxSize
        );

        Box boundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster);
};