// This file contains the entry point to start
// the ROS node "velodyneReaderNode". 
// This node is what initiates and maintains
// velodyneSocketReader or velodynePCAPReader
// listening/reading LiDAR data
#include <iostream>
#include <chrono>
#include <pcl/PCLPointCloud2.h>
#include <queue>
#include <fstream>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar/velodynePCAPReader.h"
#include "floam_cpu/laserMappingClass.h"
#include "floam_cpu/laserProcessingClass.h"
#include "floam_cpu/lidarOptimisation.h"
#include "floam_cpu/odomEstimationClass.h"
#include "lidar/velodyneSocketReader.h"
#include "object_detection_cpu/objPointCloudProcessor.h"
#include "object_detection_cpu/objKdtree.h"
#include "object_detection_cpu/objCluster.h"
#include "object_detection_cpu/objRansac.h"

arachnida::velodyneSocketReader sockRead;

// velodynePcapReader to read LiDAR data from .pcap files (recorded using WireShark) 
// std::string pcapFileName = "RoboticsLab";
// arachnida::velodynePCAPReader reader("/mnt/c/Users/lukap/OneDrive/Desktop/Study/Fifth Year/Honours/Sample Velodyne Data/" + pcapFileName + ".pcap");

// START: F-LOAM objects used for testing
// LaserMappingClass laserMapping;
// LaserProcessingClass laserProcessing;
// odomEstimationClass odomEstimation;
// objPointCloudProcessor objProcessor;
// END: F-LOAM objects used for testing

std::vector<char> frameBuffer;
std::vector<std::vector<char>> frameBufferQueue;


int main(int argc, char **argv) {
    ros::init(argc, argv, "velodyneReaderNode");
    ros::NodeHandle nh;
    std::cout << "Instantiating veloPublisher\n"; 
    ros::Publisher pcPublisher = nh.advertise<pcl::PCLPointCloud2>("arachnida/point_cloud/pcl", 100);

    ros::Rate loop_rate(10);

    // Connect to the VLP-16 via UDP socket 
    // and publish it via pcPublisher
    sockRead.connect(frameBuffer, frameBufferQueue, pcPublisher, false);

    // START: F-LOAM DEBUGGING STUFF
    // int count = 0;

    // int totalFramesProcessed = 0;
    // int totalTimeElapsed = 0;
    // bool isOdomInitialised = false;
    
    // Create files to record F-LOAM data for debugging
    // std::ofstream movementFile("movementData_" + pcapFileName + ".txt"); // this was used for debugging - remove later if not needed
	// std::ofstream translationCsv("floamTranslation_" + pcapFileName + ".csv");
	// std::ofstream rotationCsv("floamRotation_" + pcapFileName + ".csv");
    // END: F-LOAM DEBUGGING STUFF
    
    while(ros::ok()) {
    //     reader.readFile(); // return a vector of frames
    //     for(int i = 1; i < reader.getFrameClouds().size(); i++) { // loop through the frames (start at index 1 because we need to compare the current frame to the previous frame)
    //         pcl::PointCloud<pcl::PointXYZI>::Ptr frame = reader.getFrameClouds()[i];
    //         if(frame->size() < 3000) continue;


	// 		pcl::PCLPointCloud2 cloud2;
	// 		cloud2.header.seq = i;

	// 		pcl::toPCLPointCloud2(*frame, cloud2);
			
	// 		pcPublisher.publish(cloud2);
            
    //         pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudEdge(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the edge points from the frame
    //         pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudSurf(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the surface points from the frame

    //         laserProcessing.featureExtraction(frame, pointCloudEdge, pointCloudSurf); // extract the edge and surface points from the frame

    //         totalFramesProcessed++;

    //         // TODO: may need to create a pointCloudFiltered PointCloud to store the surface + edge points (laserMappingNode listens to it for some reason)
    //         std::cout << "pointCloudEdge Size:" << pointCloudEdge->size() << "points \n";
    //         std::cout << "pointCloudSurf Size:" << pointCloudSurf->size() << "points \n";

            
    //         if(pointCloudEdge->size() > 0 && pointCloudSurf->size() > 0) {
    //             if(isOdomInitialised) {
    //                 auto t1 = std::chrono::high_resolution_clock::now();
    //                 odomEstimation.updatePointsToMap(pointCloudEdge, pointCloudSurf);
    //                 auto t2 = std::chrono::high_resolution_clock::now();
    //                 auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
    //                 std::cout << "updatePointsToMap:" << duration << "us \n";
    //             } else {
    //                 odomEstimation.init(0.1);
    //                 odomEstimation.initMapWithPoints(pointCloudEdge, pointCloudSurf);
    //                 isOdomInitialised = true;
    //                 // ROS_INFO("Odom initialised");
    //             }

    //             Eigen::Quaterniond qCurrent(odomEstimation.odom.rotation());
    //             Eigen::Vector3d tCurrent = odomEstimation.odom.translation();

    //             // static tf::TransformBroadcaster br;
    //             tf::Transform transform;
    //             transform.setOrigin(tf::Vector3(tCurrent.x(), tCurrent.y(), tCurrent.z()));
    //             tf::Quaternion q(qCurrent.x(), qCurrent.y(), qCurrent.z(), qCurrent.w());
    //             transform.setRotation(q);

    //             movementFile << "Transform --------- Frame " << i << "\n";
    //             movementFile << "Translation: " << tCurrent.x() << ", " << tCurrent.y() << ", " << tCurrent.z() << "\n";
    //             movementFile << "Rotation: " << qCurrent.x() << ", " << qCurrent.y() << ", " << qCurrent.z() << ", " << qCurrent.w() << "\n\n\n";

	// 			translationCsv << tCurrent.x() << "," << tCurrent.y() << "," << tCurrent.z() << "\n";
	// 			rotationCsv << qCurrent.x() << "," << qCurrent.y() << "," << qCurrent.z() << "," << qCurrent.w() << "\n";
                
    //         }
    //         if(i > 20) break;
    //     }
    //     sensor_msgs::PointCloud2 pcFrameMsg;
       // veloPublisher.publish(rosMsg);
        // break;
        ros::spinOnce();
        loop_rate.sleep();

        // ++count;
    }

    // movementFile.close();
    // translationCsv.close();
    // rotationCsv.close();

    return 0;
}
