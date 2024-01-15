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
#include "floam_cpu/laserProcessingClass.h"
#include <chrono>

// Could make this more efficient by simply storing the ID that a particular point came from in the point cloud and then retrieving it (scanID) directly
void LaserProcessingClass::featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutEdge, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutSurfaces) {
    auto t1 = std::chrono::high_resolution_clock::now();
    std::vector<int> indices; // figure out what "indices" are later
    pcl::removeNaNFromPointCloud(*pcIn, indices);
    std::cout << "pcIn size: " << pcIn->points.size() << "\n";

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudScans; // Stores all the points obtained by a particular laser as a point cloud -> Therefore each of these point clouds contains only points along a "horizontal" line -> Leads to fewer false positives as original point cloud is sparse vertically but dense horizontally
    for(int i = 0; i < 16; i++) { // 16 because that's how many lasers the VLP16 has
        laserCloudScans.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
    }
    int invalidScanIDs = 0;
    for(int i = 0; i < (int) pcIn->points.size(); i++) { 
        if(pcIn->points[i].z == 0) continue; // if the z coordinate is zero then it will cause zero to be in the numerator of the atan function
        int scanID = 0;
        double distance = sqrt(pcIn->points[i].x*pcIn->points[i].x + pcIn->points[i].y*pcIn->points[i].y);
        // std::cout << "point: (" << pcIn->points[i].x << ", " << pcIn->points[i].y << ", " << pcIn->points[i].z << ")\n";
        if(distance > 100) continue; // 100m is the max range of the VLP16
        double angle = atan(pcIn->points[i].z / distance) * 180 / M_PI;
        
        scanID = int((angle + 15) / 2 + 0.5); // classifies the points as being part of a certain laser scan; e.g if point is from laser 0 then angle = -15deg. Angle + 15 = 0. 0/2 = 0.5. 0.5 rounded to nearest int = 0. Therefore point is from laser 0. 
        if(scanID > 15 || scanID < 0) {
            continue; // checks that the scanID is valid. If not then continues to next loop iteration
        }

        laserCloudScans[scanID]->push_back(pcIn->points[i]); 
    }

    for(int i = 0; i < 16; i++) {
        if(laserCloudScans[i]->points.size() < 131) continue;

        std::vector<Double2d> cloudCurvature;
        int totalPoints = laserCloudScans[i]->points.size() - 10; // what's the -10 for?

        // This for loop denotes equation (1) in the F-LOAM paper: https://arxiv.org/pdf/2107.00822.pdf (backed up in handover materials)
        // Eqn (LaTeX): \sigma_k^{(m,n)} = \frac{1}{|\mathcal{S}_k^{(m,n)}|} \sum\limits_{\mathbf{p}_k^{(m,j)}\in\mathcal{S}_k^{(m,n)}} (|| \mathbf{p}_k^{(m,j)} - \mathbf{p}_k^{(m,n)} ||)
        for(int j = 5; j < (int) laserCloudScans[i]->points.size() - 5; j++) { // considers 11 points at a time so has to start at 5 and end at size - 5 (e.g first points considered are: j-5, j-4, j-3, j-2, j-1, j, j+1, j+2, j+3, j+4, j+5)
            double diffX = laserCloudScans[i]->points[j - 5].x + laserCloudScans[i]->points[j - 4].x + laserCloudScans[i]->points[j - 3].x + laserCloudScans[i]->points[j - 2].x + laserCloudScans[i]->points[j - 1].x - 10*laserCloudScans[i]->points[j].x + laserCloudScans[i]->points[j + 1].x + laserCloudScans[i]->points[j + 2].x + laserCloudScans[i]->points[j + 3].x + laserCloudScans[i]->points[j + 4].x + laserCloudScans[i]->points[j + 5].x;
            double diffY = laserCloudScans[i]->points[j - 5].y + laserCloudScans[i]->points[j - 4].y + laserCloudScans[i]->points[j - 3].y + laserCloudScans[i]->points[j - 2].y + laserCloudScans[i]->points[j - 1].y - 10*laserCloudScans[i]->points[j].y + laserCloudScans[i]->points[j + 1].y + laserCloudScans[i]->points[j + 2].y + laserCloudScans[i]->points[j + 3].y + laserCloudScans[i]->points[j + 4].y + laserCloudScans[i]->points[j + 5].y;
            double diffZ = laserCloudScans[i]->points[j - 5].z + laserCloudScans[i]->points[j - 4].z + laserCloudScans[i]->points[j - 3].z + laserCloudScans[i]->points[j - 2].z + laserCloudScans[i]->points[j - 1].z - 10*laserCloudScans[i]->points[j].z + laserCloudScans[i]->points[j + 1].z + laserCloudScans[i]->points[j + 2].z + laserCloudScans[i]->points[j + 3].z + laserCloudScans[i]->points[j + 4].z + laserCloudScans[i]->points[j + 5].z;
            Double2d distance(j, diffX*diffX + diffY*diffY + diffZ*diffZ); // distance = euclidean norm = diffX*diffX + diffY*diffY + diffZ*diffZ = \sum\limits_{\mathbf{p}_k^{(m,j)}\in\mathcal{S}_k^{(m,n)}}||\mathbf{p}_k^{(m,j)} - \mathbf{p}_k^{(m,n)}||
            cloudCurvature.push_back(distance);
        }

        for(int j = 0; j < 6; j++) {
            int sectorLength = (int)(totalPoints/6);
            int sectorStart = sectorLength*j;
            int sectorEnd = sectorLength*(j+1) - 1;
            if(j == 5) {
                sectorEnd = totalPoints - 1;
            }
            std::vector<Double2d> subCloudCurvature(cloudCurvature.begin() + sectorStart, cloudCurvature.begin() + sectorEnd);
            featureExtractionFromSector(laserCloudScans[i], subCloudCurvature, pcOutEdge, pcOutSurfaces);
        }
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
    std::cout << "featureExtraction: " << duration << "us\n";

}

// This function is parallelisable
// Would be good to have a look at:
//  - the performance of this function
//  - the number of loops (i.e what is the value of cloudCurvature.size()?)
void LaserProcessingClass::featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, std::vector<Double2d> &cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutEdge, pcl::PointCloud<pcl::PointXYZI>::Ptr &pcOutSurfaces) {
    auto t1 = std::chrono::high_resolution_clock::now();
    std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const Double2d &a, const Double2d &b) {
        return a.value < b.value;
    });

    int largestPickedNum = 0;
    std::vector<int> pickedPoints;
    int pointInfoCount = 0;

    for(int i = cloudCurvature.size() - 1; i >= 0; i--) { // changing this to i > -1 might be more efficient?
        int ind = cloudCurvature[i].id;
        if(std::find(pickedPoints.begin(), pickedPoints.end(), ind) == pickedPoints.end()) {
            // std::cout << "Find: passed\n";
            if(cloudCurvature[i].value <= 0.1) break;

            // std::cout << "break: passed\n";
            largestPickedNum++;
            pickedPoints.push_back(ind);

            if(largestPickedNum <= 20) { // changing this to largestPickedNum < 21 might be more efficient?
                // std::cout << "pcOutEdge push_back: " << ind << "\n";
                pcOutEdge->push_back(pcIn->points[ind]);
                pointInfoCount++;
            } else break;

            for(int k = 1; k <= 5; k++) {
                double diffX = pcIn->points[ind + k].x - pcIn->points[ind + k - 1].x;
                double diffY = pcIn->points[ind + k].y - pcIn->points[ind + k - 1].y;
                double diffZ = pcIn->points[ind + k].z - pcIn->points[ind + k - 1].z;

                if(diffX*diffX + diffY*diffY + diffZ*diffZ > 0.05) break;
                pickedPoints.push_back(ind + k);
            }
            
            for(int k = 1; k >= 5; k--) {
                double diffX = pcIn->points[ind + k].x - pcIn->points[ind + k - 1].x;
                double diffY = pcIn->points[ind + k].y - pcIn->points[ind + k - 1].y;
                double diffZ = pcIn->points[ind + k].z - pcIn->points[ind + k - 1].z;

                if(diffX*diffX + diffY*diffY + diffZ*diffZ > 0.05) break;
                pickedPoints.push_back(ind + k);
            }
        }
    }

    for(int i = 0; i <= (int) cloudCurvature.size() - 1; i++) {
        int ind = cloudCurvature[i].id;
        if(std::find(pickedPoints.begin(), pickedPoints.end(), ind) == pickedPoints.end()) {
            pcOutSurfaces->push_back(pcIn->points[ind]);
        }
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
    // std::cout << "featureExtractionFromSector duration: " << duration << "us\n";
}

LaserProcessingClass::LaserProcessingClass() {

}

Double2d::Double2d(int idIn, double valueIn) {
    id = idIn;
    value = valueIn;
};

PointsInfo::PointsInfo(int layerIn, double timeIn) {
    layer = layerIn;
    time = timeIn;
};
