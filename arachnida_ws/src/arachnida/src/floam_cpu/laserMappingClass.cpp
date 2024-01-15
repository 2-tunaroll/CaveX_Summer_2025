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

#include "floam_cpu/laserMappingClass.h"

void LaserMappingClass::init(double mapResolution) {
    for(int i = 0; i < LASER_CELL_RANGE_HORIZONTAL*2+1; i++) {
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> mapHeightTemp;
        for(int j = 0; j < LASER_CELL_RANGE_HORIZONTAL*2+1; j++) {
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> mapDepthTemp;
            for(int k=0; k < LASER_CELL_RANGE_VERTICAL*2+1; k++) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudTemp(new pcl::PointCloud<pcl::PointXYZI>);
                mapDepthTemp.push_back(pointCloudTemp);
            }
            mapHeightTemp.push_back(mapDepthTemp);
        }
        map.push_back(mapHeightTemp);
    }

    originInMapX = LASER_CELL_RANGE_HORIZONTAL;
    originInMapY = LASER_CELL_RANGE_HORIZONTAL;
    originInMapZ = LASER_CELL_RANGE_VERTICAL;
    mapWidth = LASER_CELL_RANGE_HORIZONTAL*2+1;
    mapHeight = LASER_CELL_RANGE_HORIZONTAL*2+1;
    mapDepth = LASER_CELL_RANGE_HORIZONTAL*2+1;

    // downsampling size
    downSampledFilter.setLeafSize(mapResolution, mapResolution, mapResolution);
}

void LaserMappingClass::addWidthCellNegative() {
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> mapHeightTemp;
    for(int j = 0; j < mapHeight; j++) {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> mapDepthTemp;
        for(int k = 0; k < mapDepth; k++) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudTemp;
            mapDepthTemp.push_back(pointCloudTemp);
        }
        mapHeightTemp.push_back(mapDepthTemp);
    }
    map.insert(map.begin(), mapHeightTemp);

    originInMapX++;
    mapWidth++;
}

void LaserMappingClass::addWidthCellPositive() {
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> mapHeightTemp;
    for(int j = 0; j < mapHeight; j++) {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> mapDepthTemp;
        for(int k = 0; k < mapDepth; k++) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudTemp;
            mapDepthTemp.push_back(pointCloudTemp);
        }
        mapHeightTemp.push_back(mapDepthTemp);
    }

    map.push_back(mapHeightTemp);
    mapWidth++;
}

void LaserMappingClass::addHeightCellNegative() {
    for(int i = 0; i < mapWidth; i++) {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> mapDepthTemp;
        for(int k = 0; k < mapDepth; k++) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudTemp;
            mapDepthTemp.push_back(pointCloudTemp);
        }
        map[i].insert(map[i].begin(), mapDepthTemp);
    }
    originInMapY++;
    mapHeight++;
}

void LaserMappingClass::addHeightCellPositive() {
    for(int i = 0; i < mapWidth; i++) {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> mapDepthTemp;
        for(int k = 0; k < mapDepth; k++) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudTemp;
            mapDepthTemp.push_back(pointCloudTemp);
        }
        map[i].push_back(mapDepthTemp);
    }
    mapHeight++;
}

void LaserMappingClass::addDepthCellNegative() {
    for(int i = 0; i < mapWidth; i++) {
        for(int j = 0; j < mapHeight; j++) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudTemp;
            map[i][j].insert(map[i][j].begin(), pointCloudTemp);
        }
    }
    originInMapZ++;
    mapDepth++;
}

void LaserMappingClass::addDepthCellPositive() {
    for(int i = 0; i < mapWidth; i++) {
        for(int j = 0; j < mapHeight; j++) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudTemp;
            map[i][j].push_back(pointCloudTemp);
        }
    }
    mapDepth++;
}

void LaserMappingClass::checkPoints(int &x, int &y, int &z) {
    while(x + LASER_CELL_RANGE_HORIZONTAL > mapWidth - 1) {
        addWidthCellPositive();
    }

    while(x - LASER_CELL_RANGE_HORIZONTAL < 0) {
        addWidthCellNegative();
        x++;
    }

    while(y + LASER_CELL_RANGE_HORIZONTAL > mapHeight - 1) {
        addHeightCellPositive();
    }

    while(y - LASER_CELL_RANGE_HORIZONTAL > mapHeight - 1) {
        addHeightCellNegative();
        y++;
    }

    while(z + LASER_CELL_RANGE_VERTICAL > mapDepth - 1) {
        addDepthCellPositive();
    }

    while(z - LASER_CELL_RANGE_VERTICAL < 0) {
        addDepthCellNegative();
        z++;
    }

    for(int i = x - LASER_CELL_RANGE_HORIZONTAL; i < x + LASER_CELL_RANGE_HORIZONTAL + 1; i++) {
        for(int j = y - LASER_CELL_RANGE_HORIZONTAL; j < y + LASER_CELL_RANGE_HORIZONTAL + 1; j++) {
            for(int k = z - LASER_CELL_RANGE_VERTICAL; k < z + LASER_CELL_RANGE_VERTICAL + 1; k++) {
                if(map[i][j][k] == NULL) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudTemp(new pcl::PointCloud<pcl::PointXYZI>());
                    map[i][j][k] = pointCloudTemp;
                }
            }
        }
    }
}

void LaserMappingClass::updateCurrentPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, const Eigen::Isometry3d &poseCurrent) {
    int currentPosIdX = int(std::floor(poseCurrent.translation().x() / LASER_CELL_WIDTH + 0.5)) + originInMapX;
    int currentPosIdY = int(std::floor(poseCurrent.translation().y() / LASER_CELL_HEIGHT + 0.5)) + originInMapY;
    int currentPosIdZ = int(std::floor(poseCurrent.translation().z() / LASER_CELL_DEPTH + 0.5)) + originInMapZ;

    checkPoints(currentPosIdX, currentPosIdY, currentPosIdZ);

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedPointCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*pcIn, *transformedPointCloud, poseCurrent.cast<float>());

    // Saving points
    for(int i = 0; i < (int) transformedPointCloud->points.size(); i++) {
        pcl::PointXYZI pointTemp = transformedPointCloud->points[i];

        // For visualisation only
        pointTemp.intensity = std::min(1.0, std::max(pcIn->points[i].z+2.0, 0.0) / 5);
        int currentPointIdX = int(std::floor(pointTemp.x / LASER_CELL_WIDTH + 0.5)) + originInMapX;
        int currentPointIdY = int(std::floor(pointTemp.y / LASER_CELL_WIDTH + 0.5)) + originInMapY;
        int currentPointIdZ = int(std::floor(pointTemp.z / LASER_CELL_WIDTH + 0.5)) + originInMapZ;

        map[currentPointIdX][currentPointIdY][currentPointIdZ]->push_back(pointTemp);
    }

    // Filtering points
    for(int i = currentPosIdX - LASER_CELL_RANGE_HORIZONTAL; i < currentPosIdX + LASER_CELL_RANGE_HORIZONTAL + 1; i++) {
        for(int j = currentPosIdY - LASER_CELL_RANGE_HORIZONTAL; j < currentPosIdY + LASER_CELL_RANGE_HORIZONTAL + 1; j++) {
            for(int k = currentPosIdZ - LASER_CELL_RANGE_VERTICAL; k < currentPosIdZ + LASER_CELL_RANGE_VERTICAL + 1; k++) {
                downSampledFilter.setInputCloud(map[i][j][k]);
                downSampledFilter.filter(*(map[i][j][k]));
            }
        }
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LaserMappingClass::getMap() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i = 0; i < mapWidth; i++) {
        for(int j = 0; j < mapHeight; j++) {
            for(int k = 0; k < mapDepth; k++) {
                if(map[i][j][k] != NULL) {
                    *laserCloudMap += *(map[i][j][k]);
                }
            }
        }
    }
    return laserCloudMap;
}

LaserMappingClass::LaserMappingClass() {

}