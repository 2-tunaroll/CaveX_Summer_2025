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
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "floam_cpu/laserProcessingClass.h"

LaserProcessingClass laserProcessor;
std::mutex mutexLock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;

ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLaserCloudFiltered;

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    mutexLock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutexLock.unlock();
}

double totalTime = 0;
int totalFrame = 0;

void laserProcessing() {
    while(true) {
        if(!pointCloudBuf.empty()) {
            mutexLock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudIn(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointCloudIn);
            ros::Time pointCloudTime = (pointCloudBuf.front())->header.stamp;
            pointCloudBuf.pop();
            mutexLock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudEdge(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudSurf(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            laserProcessor.featureExtraction(pointCloudIn, pointCloudEdge, pointCloudSurf);
            end = std::chrono::system_clock::now();

            std::chrono::duration<double> elapsedSeconds = end - start;
            totalFrame++;

            float timeTemp = elapsedSeconds.count() * 1000;
            totalTime += timeTemp;

            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudFiltered(new pcl::PointCloud<pcl::PointXYZI>());
            *pointCloudFiltered += *pointCloudEdge;
            *pointCloudFiltered += *pointCloudSurf;
            pcl::toROSMsg(*pointCloudFiltered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointCloudTime;
            laserCloudFilteredMsg.header.frame_id = "base_link";
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointCloudEdge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointCloudTime;
            edgePointsMsg.header.frame_id = "base_link";
            pubEdgePoints.publish(edgePointsMsg);

            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointCloudSurf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointCloudTime;
            surfPointsMsg.header.frame_id = "base_link";
            pubSurfPoints.publish(surfPointsMsg);
        }

        // sleep 2ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    double verticalAngle = 2.0;
    double scanPeriod = 0.1;
    double maxDis = 60.0;
    double minDis = 2.0;

    // ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyneHandler); // not needed

    // pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_filtered", 100); // probably not needed

    // pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100); // probably not needed

    // pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100); // probably not needed

    std::thread laserProcessingProcess{laserProcessing};
    
    ros::spin();
    return 0;
}
