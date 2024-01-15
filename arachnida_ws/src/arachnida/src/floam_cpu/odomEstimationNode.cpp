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
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "floam_cpu/odomEstimationClass.h"

odomEstimationClass odomEstimation;
std::mutex mutexLock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;

ros::Publisher pubLaserOdometry;

void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    mutexLock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutexLock.unlock();
}

void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    mutexLock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutexLock.unlock();
}

bool isOdomInitialized = false;
double totalTime = 0;
int totalFrame = 0;

void odomEst() {
    while(true) {
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()) {
            
            //Read data
            mutexLock.lock();
            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec() < pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5 * 0.1)) { // 0.1 = scan period (100ms)
                pointCloudSurfBuf.pop();
                mutexLock.unlock();
                continue;
            }

            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec() < pointCloudSurfBuf.front()->header.stamp.toSec()-0.5 * 0.1)) { // 0.1 = scan period (100ms)
                pointCloudEdgeBuf.pop();
                mutexLock.unlock();
                continue;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudSurfIn(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudEdgeIn(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointCloudEdgeIn);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointCloudSurfIn);
            ros::Time pointCloudTime = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutexLock.unlock();

            if(!isOdomInitialized) {
                odomEstimation.initMapWithPoints(pointCloudEdgeIn, pointCloudSurfIn);
                isOdomInitialized = true;
                ROS_INFO("Odom Initialised");
            } else {
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                odomEstimation.updatePointsToMap(pointCloudEdgeIn, pointCloudSurfIn);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsedSeconds = end - start;
                totalFrame++;
                float timeTemp = elapsedSeconds.count() * 100;
                totalTime += timeTemp;
                ROS_INFO("Average odom estimation time %f ms \n \n", totalTime/totalFrame);
            }

            Eigen::Quaterniond qCurrent(odomEstimation.odom.rotation());
            Eigen::Vector3d tCurrent = odomEstimation.odom.translation();

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(tCurrent.x(), tCurrent.y(), tCurrent.z()));
            tf::Quaternion q(qCurrent.x(), qCurrent.y(), qCurrent.z(), qCurrent.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, pointCloudTime, "map", "base_link"));

            // Publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map";
            laserOdometry.child_frame_id = "base_link";
            laserOdometry.header.stamp = pointCloudTime;
            laserOdometry.pose.pose.orientation.x = qCurrent.x();
            laserOdometry.pose.pose.orientation.y = qCurrent.y();
            laserOdometry.pose.pose.orientation.z = qCurrent.z();
            laserOdometry.pose.pose.orientation.w = qCurrent.w();
            laserOdometry.pose.pose.position.x = tCurrent.x();
            laserOdometry.pose.pose.position.y = tCurrent.y();
            laserOdometry.pose.pose.position.z = tCurrent.z();
            pubLaserOdometry.publish(laserOdometry);
        }

        // sleep 2ms every time
        std::chrono::milliseconds sleepDuration(2);
        std::this_thread::sleep_for(sleepDuration);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odomEstimationNode");
    ros::NodeHandle nh;

    odomEstimation.init(0.4); // mapResolution = 0.4
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_surf", 100, velodyneSurfHandler);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    std::thread odomEstimationProcess{odomEst};

    ros::spin();

    return 0;
}