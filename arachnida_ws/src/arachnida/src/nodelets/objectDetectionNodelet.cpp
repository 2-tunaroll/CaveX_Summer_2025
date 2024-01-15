// This nodelet runs the object detection
// code upon receiving live LiDAR data
// on the arachnida/point_cloud/pcl
// topic.
#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>


// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Object Detection includes
#include "object_detection_cpu/objBox.h"
#include "object_detection_cpu/objKdtree.h"
#include "object_detection_cpu/objRansac.h"
#include "object_detection_cpu/objRender.h"
#include "object_detection_cpu/objCluster.h"
#include "object_detection_cpu/objPointCloudProcessor.h"

// Obstacle messages
#include "arachnida/Obstacle.h"
#include "arachnida/ObstacleList.h"

// Temp message used for ROSNodeJS
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

namespace arachnida {
	class ObjectDetectionNodelet : public nodelet::Nodelet {
		public:
			ObjectDetectionNodelet() {}
		private:
			virtual void onInit() {
				ROS_INFO("[objectDetectionNodelet.cpp] Initializing Object Detection nodelet...");
				ros::NodeHandle &nh = getNodeHandle();
				pcSub = nh.subscribe("arachnida/point_cloud/pcl", 100, &ObjectDetectionNodelet::cloudCallback, this);
				obstaclesDetectedPub = nh.advertise<arachnida::ObstacleList>("arachnida/object_detection/objects_detected", 100);
				// obstaclesDetectedPub = nh.advertise<sensor_msgs::PointCloud>("arachnida/object_detection/objects_detected", 100);
				ROS_INFO("[obstacleDetectionNodelet.cpp] Initialized Obstacle Detection nodelet...");
			};
			

			void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {

            
                // sensor_msgs::PointCloud ingenuityArachnidaLive_obsMsg; // temp message for obstacles to work with ROSNodeJS
                // ingenuityArachnidaLive_obsMsg.header.seq = cloud_msg->header.seq;
                // ingenuityArachnidaLive_obsMsg.header.frame_id = cloud_msg->header.frame_id;

				arachnida::ObstacleList obsMsg;

				// arachnida::ObstacleListConstPtr obsMsgConstPtr;
				obsMsg.header.seq = cloud_msg->header.seq;
				obsMsg.header.frame_id = cloud_msg->header.frame_id;

                std::string sequenceStr = std::to_string(obsMsg.header.seq);
                const char *sequenceStrArray = sequenceStr.c_str();
                if(sequenceStrArray[sequenceStr.size() - 1] == '5') { // Throttling F-LOAM rate to 1Hz by only letting it run when the sequence number ends in 5 (which should happen once per second since the arachnida/point_cloud/pcl topic is published to every 100ms)
    //             auto millisSinceLastObjDetect = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastObjectDetectionTimestamp);
				// if(millisSinceLastObjDetect.count() > 1000) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr pcFrame(new pcl::PointCloud<pcl::PointXYZI>());
                    pcl::fromROSMsg(*cloud_msg, *pcFrame);

					lastObjectDetectionTimestamp = std::chrono::high_resolution_clock::now(); // Reset the last obj detection timestamp
            
					pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
					pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());
					pcl::PointCloud<pcl::PointXYZI>::Ptr pcFilter(new pcl::PointCloud<pcl::PointXYZI>());
			
					Eigen::Vector4f minVec = Eigen::Vector4f(-10, -6.2, -2, 1);
					Eigen::Vector4f maxVec = Eigen::Vector4f(15, 7, 10, 1);

					pcFilter = objProcessor.filterCloud(pcFrame, 0.1, minVec, maxVec);

					std::unordered_set<int> inliers = ransacPlane(pcFilter, 10, 0.2);


					for(int index = 0; index < pcFilter->points.size(); index++) {
						pcl::PointXYZI point = pcFilter->points[index];

						if(inliers.count(index)) {
							pointCloudInliers->points.push_back(point);
						} else {
							pointCloudOutliers->points.push_back(point);
						}
					}
		
					KdTree *tree = new KdTree;
					std::vector<std::vector<float>> pointVectors;

					for(int j = 0; j < pointCloudOutliers->points.size(); j++) {
						std::vector<float> pointVector;
						pointVector.push_back(pointCloudOutliers->points[j].x);
						pointVector.push_back(pointCloudOutliers->points[j].y);
						pointVector.push_back(pointCloudOutliers->points[j].z);
						pointVectors.push_back(pointVector);
						tree->insert(pointVector, j);
					}

					std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = euclideanCluster(pointVectors, tree, 0.1, 20);

					std::cout << "[objectDectionNodelet.cpp] pcFrame size: " << pcFrame->points.size() << "\n";
					std::cout << "[objectDectionNodelet.cpp] pointCloudInliers size: " << pointCloudInliers->points.size() << "\n";
					std::cout << "[objectDectionNodelet.cpp] pointCloudOutliers size: " << pointCloudOutliers->points.size() << "\n";
					std::cout << "[objectDectionNodelet.cpp] pointVectors size: " << pointVectors.size() << "\n";
					std::cout << "[objectDectionNodelet.cpp] pcFilter size: " << pcFilter->size() << "\n";
					std::cout << "[objectDectionNodelet.cpp] clusters size: " << clusters.size() << " clusters\n";
					
					int clusterID = 1;
					for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters) {
						std::cout << "[objectDectionNodelet.cpp] cluster size: " << cluster->size() << " points\n";
						Box box = objProcessor.boundingBox(cluster);
						arachnida::Obstacle ob;
						ob.id = clusterID;
						ob.x = box.x_min + (box.x_max - box.x_min) / 2;
						ob.y = box.y_min + (box.y_max - box.y_min) / 2;
						ob.z = box.z_min + (box.z_max - box.z_min) / 2;
						ob.radius = (box.x_max - box.x_min) / 2;

                        ob.verticesX = { box.x_min, box.x_max };
                        ob.verticesY = { box.y_min, box.y_max };
                        ob.verticesZ = { box.z_min, box.z_max };

						obsMsg.obstacles.push_back(ob);

                        
                        // geometry_msgs::Point32 IDPoint;
                        // IDPoint.x = (float) clusterID;
                        // IDPoint.y = 0;
                        // IDPoint.z = (box.x_max - box.x_min) / 2;
                        //
                        // geometry_msgs::Point32 dataPoint;
                        // dataPoint.x = box.x_min + (box.x_max - box.x_min) / 2;
                        // dataPoint.y = box.y_min + (box.y_max - box.y_min) / 2;
                        // dataPoint.z = box.z_min + (box.z_max - box.z_min) / 2;

                    
                        // ingenuityArachnidaLive_obsMsg.points.push_back(dataPoint);

						clusterID++;
					}
                    obstaclesDetectedPub.publish(obsMsg);
                    // obstaclesDetectedPub.publish(ingenuityArachnidaLive_obsMsg);
				} else {
					ROS_INFO("[obstacleDetectionNodelet.cpp] Throttling");
				}

				ROS_INFO("[objectDectionNodelet.cpp] Received point cloud message");

			};

			std::chrono::time_point<std::chrono::high_resolution_clock> lastObjectDetectionTimestamp = std::chrono::high_resolution_clock::now(); // Stores the last time object detection was run - Used for throttling
			ros::Subscriber pcSub;
			ros::Publisher obstaclesDetectedPub;

			objPointCloudProcessor objProcessor;

	};

	PLUGINLIB_EXPORT_CLASS(arachnida::ObjectDetectionNodelet, nodelet::Nodelet);
};

