#include <chrono>
#include <string>
#include <iostream>
#include <vector>

#include "objKdtree.h"

void createCluster(
    const std::vector<std::vector<float>> &points, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster, 
    int *flag, 
    KdTree *tree, 
    float distanceTolerance, 
    int i
);

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> euclideanCluster(
    const std::vector<std::vector<float>> &points, 
    KdTree *tree, 
    float distanceTolerance, 
    int minSize
);