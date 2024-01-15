#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <unordered_set>

std::unordered_set<int> ransacPlane(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
    const int maxIterations, 
    const float distanceTolerance
);