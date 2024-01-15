#include "object_detection_cpu/objPointCloudProcessor.h"

objPointCloudProcessor::objPointCloudProcessor() {

}

objPointCloudProcessor::~objPointCloudProcessor() {

}



void objPointCloudProcessor::numPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    std::cout << cloud->points.size() << "\n";
}

pcl::PointCloud<pcl::PointXYZI>::Ptr objPointCloudProcessor::filterCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
    const float filterRes, 
    Eigen::Vector4f minPoint, 
    Eigen::Vector4f maxPoint
) {
    auto startTime = std::chrono::steady_clock::now();

    pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZI>());

    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.filter(*cloudFiltered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filterCloud took " << elapsedTime.count() << " milliseconds\n";

    return cloudFiltered;
}

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> objPointCloudProcessor::separateClouds(
    pcl::PointIndices::Ptr inliers, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud
) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>());

    for(int index : inliers->indices) {
        planeCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult(obstacleCloud, planeCloud);

    return segResult;
}

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> objPointCloudProcessor::segmentPlane(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
    const int maxIterations, 
    const float distanceThreshold
) {
    auto startTime = std::chrono::steady_clock::now();

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0) {
        std::cout << "[objPointCloudProcessor] Could not estimate a planar model for the given dataset\n";
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "segmentPlane took " << elapsedTime.count() << " milliseconds\n";
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = separateClouds(inliers, cloud);
    return segResult;
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> objPointCloudProcessor::clusterCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
    const float clusterTolerance, 
    const int minSize, 
    const int maxSize
) {
    auto startTime = std::chrono::steady_clock::now();

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZI>());
    kdTree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(kdTree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices pIndices : clusterIndices) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZI>());

        for(int index : pIndices.indices) {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clusterCloud took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters\n";
    return clusters;
}

Box objPointCloudProcessor::boundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster) {
    pcl::PointXYZI minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}