#include "object_detection_cpu/objCluster.h"

void createCluster(
    const std::vector<std::vector<float>> &points,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster, 
    int *flag, 
    KdTree *tree, 
    float distanceTolerance, 
    int i
) {
    if(flag[i] == 1) return;
    flag[i] = 1;

    pcl::PointXYZI z;

    z.x = points[i][0];
    z.y = points[i][1];
    z.z = points[i][2];
    z.intensity = 0.0f;

    cluster->points.push_back(z);

    std::vector<int> nearPoint = tree->search(points[i], distanceTolerance);

    for(int j = 0; j < nearPoint.size(); j++) {
        if(flag[nearPoint[j]] == 0) createCluster(points, cluster, flag, tree, distanceTolerance, nearPoint[j]);
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> euclideanCluster(
    const std::vector<std::vector<float>> &points, 
    KdTree *tree, 
    float distanceTolerance, 
    int minSize
) {
    int size = points.size();
    int flag[size];    
    
    auto t1 = std::chrono::high_resolution_clock::now();

    // Add point to KD Tree and populate flag array with zeroes
    for(int i = 0; i < size; i++) {
        tree->insert(points[i], i);
        flag[i] = 0;
    }

    // Vector of pointcloud pointers
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

    // Populate clusters vector
    for(int i = 0; i < size; i++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);

        if(flag[i] == 0) {
            createCluster(points, cluster, flag, tree, distanceTolerance, i);
            if(cluster->points.size() >= minSize) clusters.push_back(cluster);
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    std::chrono::duration<double, std::milli> ms_double = t2 - t1;
    std::cout << "cluster duration: " << dur.count() << "ms\n";

    return clusters;
}
