#define M_PI 3.14159274101257324219

#include <chrono>
#include <cmath>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "arachnida/lidar/velodynePCAPReader.h"
#include "arachnida/lidar/velodyneUtils.h"
#include "floam_cpu/laserProcessingClass.h"
#include "object_detection_cpu/objPointCloudProcessor.h"
#include "object_detection_cpu/objRansac.h"
#include "object_detection_cpu/objCluster.h"
#include "object_detection_cpu/objRender.h"
#include "object_detection_cpu/objBox.h"


    arachnida::velodynePCAPReader::velodynePCAPReader(std::string absolutePath) : pointCloud(new pcl::PointCloud<pcl::PointXYZI>) {
        this->absolutePath = absolutePath;
    }

    // Returns the vertical angle of a laser relative to the VLP16's horizon in radians from a laser ID/channel ID
    double getLaserAngleFromLaserID(int channelID) {
        if(channelID < 1 || channelID > 32) {
            std::cout << "1\n";
            return 0;
        }
        int laserID = channelID - 1;
        if(laserID > 15) laserID -= 16;
        if(laserID < 0 || laserID > 15) {
            std::cout << "2\n";
            std::cout << "laserID: " << laserID << "\n";
            return 0; // none of the lasers are angled at 0 degrees, meaning this can be used to detect an incorrect input
        }
        else if(laserID == 0) return -0.261799;
        else if(laserID == 1) return 0.0174533;
        else if(laserID == 2) return -0.226893;
        else if(laserID == 3) return 0.0523599;
        else if(laserID == 4) return -0.191986;
        else if(laserID == 5) return 0.0872665;
        else if(laserID == 6) return -0.15708;
        else if(laserID == 7) return 0.122173;
        else if(laserID == 8) return -0.122173;
        else if(laserID == 9) return 0.15708;
        else if(laserID == 10) return -0.0872665;
        else if(laserID == 11) return 0.191986;
        else if(laserID == 12) return -0.0523599;
        else if(laserID == 13) return 0.226893;
        else if(laserID == 14) return -0.0174533;
        else if(laserID == 15) return 0.261799;
        else return 0; // none of the lasers are angled at 0 degrees, meaning this can be used to detect an incorrect input
    }

    // currently for single return mode of VLP-16
    void arachnida::velodynePCAPReader::readFile() {

        objPointCloudProcessor objProcessor;
        LaserProcessingClass laserProcessing;
        auto t1 = std::chrono::high_resolution_clock::now();
        std::ifstream pcapStream(absolutePath, std::fstream::binary | std::fstream::in);
        std::streampos fsize = 0;
        fsize = pcapStream.tellg();
        pcapStream.seekg(0, std::ios::end); // go to end of file
        fsize = pcapStream.tellg() - fsize; // calculate size of file in bytes
        pcapStream.seekg(0, std::ios::beg); // go back to start of file

        char* buffer = new char[fsize];
 
        int curPos = 0;
        pcapStream.read(buffer, fsize);
        
        // // TESTING velodyneUtils.cpp
        // auto startTime = std::chrono::high_resolution_clock::now();
        // std::cout << "fsize: " << fsize << "\n";
        // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> frameClouds2; // FOR TESTING WITH VIEWER
        // pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr frameCloud(new pcl::PointCloud<pcl::PointXYZI>); // FOR TESTING WITH VIEWER
        // for(int a = 0; a < fsize - 1; a++) {
        //     if(buffer[a] == '\xFF' && buffer[a+1] == '\xEE' && a-1 > -1 && a-2 > -1) {
        //         std::vector<char> packetBuffer;
        //         int packetEndIndex = a+1247;
        //         for(int b = a; b < packetEndIndex; b++) {
        //             packetBuffer.push_back(buffer[b]);
        //         }

        //         // TODO: Need to get it to parse them into frames rather than individual packets
        //         parsePacketToPointCloud(packetBuffer, frameCloud); 
        //         if(frameCloud->points.size() > 29000) { // TODO: this isn't the correct way to construct a frame - need to use the packet timestamps and calulations instead
        //             frameClouds2.push_back(frameCloud); 
        //             frameCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>); // FOR TESTING WITH VIEWER
        //         }

        //         // parsePacketToPointCloud(packetBuffer, pointCloud);
        //         a += 1247;
        //     }
        // }
        // auto endTime = std::chrono::high_resolution_clock::now();
        // std::cout << "time taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() << "ms\n";

        // // for(pcl::PointCloud<pcl::PointXYZI>::Ptr c : frameClouds2) {
        // //     std::cout << "frameCloud: " << c->points.size() << " points\n";
        // // }

        // // std::cout << "frame count: " << frameClouds2.size() << "\n";


        // return;
        // // END: TESTING velodyneUtils.cpp

        int bytesFromStart = 0; // number of bytes looped over since start of packet (starting at 0xFF)
        bool ffFlag = false; // set to true when 0xFF byte is located so that we can test if the next byte is 0xEE (as per the VLP-16's packet structure)
        bool azimuthFlag = false; // set to true when the two-byte 0xFFEE flag is located as the following 2 bytes comprise the azimuth
        unsigned int firstPacketInBlockTimestamp = 0;
        velodyneVLP16FrameDataBlocks curFrameBlocks; // stores all the data blocks of a frame - each frame should be approixmately 100,000 bytes (well, 100,000 bytes including blocks of nullbytes between packets in raw data)
        
        // std::ofstream distanceBytesFile("distanceBytes.txt");
        
        for(int i = 0; i < fsize; i++) {
            if(i < fsize) {
                // if(i > 400000) break; // this is only here so that the loop doesn't keep going through the whole file for testing 
                if(buffer[i] == '\xFF') {
                    ffFlag = true;
                }
                else if(buffer[i] == '\xEE') {
                    if(ffFlag) { // if ffFlag is true then we must be at the start of a datablock (0xFFEE)
                        ffFlag = false;
                        if(i-2 > -1 && i-3 > -1 && buffer[i-2] == '\x00' && buffer[i-3] == '\x00') {
                            velodyneVLP16Packet curPacket;
                            // distanceBytesFile << "\n\nPacket " << packets.size()+1 << "\n\n";
                        
                            // velodyneVLP16DataBlock db;

                            // at the moment i is the index of the 0xEE byte
                            // so the next two bytes will be the azimuth bytes
                            // then after that we have 12 channels of 3 bytes each
                            // doing i += 3 once gets us to the first byte in the first channel
                            // then i += 3 gets us to the next channels (by doing it multiple times)
                            // keep count of how many times we've done this so we know when to stop
                            // unsigned char azimuthByte1 = buffer[i+1];
                            // unsigned char azimuthByte2 = buffer[i+2];
                            // db.azimuth = ((float)(azimuthByte2 << 8 | azimuthByte1))/100; // combining azimuth bytes in reverse order as int to get azimuth*100 as an integer, then devide y 100 to get true azimuth as an angle from 0 to 359.99deg
                            for(int datablock = 0; datablock < 12; datablock++) { // this is currently broken as we need to take the 0xFFEE and azimuth bytes in each datablock into account
                                // distanceBytesFile << "\n";
                                // need to get azimuth bytes and account for 0xFFEE bytes here as this code only runs when we go to a new datablock
                                if(datablock > 0) i += 4; // add 4 to i to get to the 0xEE byte (adding 3 to get from first dist byte of last channel to 0xFF byte of 0xFFEE bytes, then add another 1 to get to the 0xEE byte)
                                
                                velodyneVLP16DataBlock db;
                                unsigned char aziByte1 = buffer[i+1];
                                unsigned char aziByte2 = buffer[i+2];
                                db.azimuth = ((float)(aziByte2 << 8 | aziByte1))/100; // combining azimuth bytes in reverse order as int to get azimuth*100 as an integer, then devide y 100 to get true azimuth as an angle from 0 to 359.99deg
                                for(int channel = 0; channel < 32; channel++) {
                                    i += 3; // now i is the first byte of the (channel+1) channel (e.g if channel is 0, i is the first byte of channel 1)
                                    // std::cout << "point: " << pointCounter+(datablock*channel) << "\n";
                                    velodyneVLP16Point point;
                                
                                    unsigned char distByte1 = buffer[i];
                                    unsigned char distByte2 = buffer[i+1];
                                    unsigned char reflectivityByte = buffer[i+2];
                                
                                    point.channel = channel+1;
                                    point.distance = (float) (((float)(distByte2 << 8 | distByte1))/500); // distance is in mm, so divide by 500 to get distance in m
                                    point.reflectivity = (float) reflectivityByte; // TODO: not sure if reflectivity is a float or int
                                    db.points.push_back(point);

                                    // next thing I need to do is convert a series of packets into a frame (however many packets it takes to cover 100ms since the VLP16's motor spins at 10Hz)
                                    // to do this I need to parse the timestamp from the packet as well and when the difference between the current timestamp and the last starting packet's timestamp exceeds 100ms then we know a frame has been collected
                                }

                                curPacket.dataBlocks.push_back(db);
                            }
                            // Get timestamp (an unsigned in composed of the 4 bytes after the final reflectivity byte in the final data block)
                            // - Timestamp is the time of the first laser firing in the first data block of the packet (as per VLP-16 User Manual: https://velodynelidar.com/wp-content/uploads/2019/12/63-9243-Rev-E-VLP-16-User-Manual.pdf)
                            // Current i is @ the final reflectivity byte of the final data block, so we need the i+1, i+2, i+3, and i+4 bytes
                            i += 4;
                            unsigned char timeByte1 = buffer[i-1];
                            unsigned char timeByte2 = buffer[i];
                            unsigned char timeByte3 = buffer[i+1];
                            unsigned char timeByte4 = buffer[i+2];

                            unsigned int timestamp = timeByte4 << 24 | (timeByte3 << 16) | (timeByte2 << 8) | timeByte1;

                            // Go through each data block in curPacket, calculate its timestamp, check if 100ms as passed
                            int blockCounter = 0;
                            for(velodyneVLP16DataBlock block : curPacket.dataBlocks) {
                                unsigned int blockTimestamp = timestamp + blockCounter*55.296; // 55.296us per firing sequence
                                if(blockTimestamp - firstPacketInBlockTimestamp > 99999) { // if block timestamp is at least 100ms (100,000us) after the timestamp of the first packet int he frame then we need to start a new frame
                                    firstPacketInBlockTimestamp = blockTimestamp; // reset firstBlockInFrameTimestamp
                                    frameDataBlocks.push_back(curFrameBlocks); // add current frame struct to frames vector
                                    velodyneVLP16FrameDataBlocks newFrameBlocks; // create next frame struct
                                    curFrameBlocks = newFrameBlocks; // assign next frame struct to current frame struct

                                    curFrameBlocks.dataBlocks.push_back(block); // add block to new frame struct's data blocks vector
                                } else {
                                    // curPacket.timestamp = timestamp;
                                    curFrameBlocks.dataBlocks.push_back(block);
                                }
                                blockCounter++;
                            }
                            packets.push_back(curPacket);
                        }
                    } 
                } 
            }
            // packets.push_back(curPacket);
        }
        // distanceBytesFile.close();

        int fBlockCount = 1;
        int tooSmallCount = 0;
        int pointsConverted = 0;
        // std::ofstream pointsFile("points.txt"); // this was used for debugging - remove later if not needed
        for(velodyneVLP16FrameDataBlocks fBlock : frameDataBlocks) {
            // std::cout << "frameDataBlocks," << fBlockCount << ": " << fBlock.dataBlocks.size() << " data blocks\n";
            if(fBlock.dataBlocks.size() < 750) tooSmallCount++;
            // pointsFile << "frameDataBlocks," << fBlockCount << ": " << fBlock.dataBlocks.size() << " data blocks\n\n";
            fBlockCount++;
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr curFrameCloud(new pcl::PointCloud<pcl::PointXYZI>);

            for(velodyneVLP16DataBlock fBlockDB : fBlock.dataBlocks) {
                for(velodyneVLP16Point p : fBlockDB.points) {
                    double laserAngle = getLaserAngleFromLaserID(p.channel);
                    // std::cout << "channel: " << p.channel << "\n";
                    // std::cout << "laserAngle: " << laserAngle << "deg \n";
                    if(laserAngle != 0) {
                        // std::cout << "pointsConverted: " << pointsConverted << "\n";
                        // std::cout << "laserAngle: " << laserAngle << "rad \n";
                        // std::cout << "cos(laserAngle): " << laserAngle << "\n";
                        // std::cout << "azimuth: " << fBlockDB.azimuth << "deg \n";
                        // std::cout << "azimuth: " << fBlockDB.azimuth*(M_PI/180) << "rad \n";
                        // std::cout << "sin(azimuth): " << std::sin(fBlockDB.azimuth*(M_PI/180)) << "\n";
                        // std::cout << "cos(azimuth): " << std::cos(fBlockDB.azimuth*(M_PI/180)) << "\n";
                        double x = p.distance*std::cos(laserAngle)*std::sin(fBlockDB.azimuth*(M_PI/180));
                        double y = p.distance*std::cos(laserAngle)*std::cos(fBlockDB.azimuth*(M_PI/180));
                        double z = p.distance*std::sin(laserAngle);
                        // pointsFile << x << "\t\t" << y << "\t\t" << z << "\t\t\t\t" << p.distance << "\t\t" << (std::atan(z/p.distance)*(180/M_PI)) << "\n";
                        // std::cout << "r: " << p.distance << "\n";
                        pcl::PointXYZI cartesianPoint;
                        cartesianPoint.x = x;
                        // std::cout << "x: " << cartesianPoint.x << "\n";
                        cartesianPoint.y = y;
                        // std::cout << "y: " << cartesianPoint.y << "\n";
                        cartesianPoint.z = z;
                        // std::cout << "z: " << cartesianPoint.z << "\n";
                        cartesianPoint.intensity = p.reflectivity;
                        // std::cout << "ref: " << cartesianPoint.intensity << "\n";
                        
                        pointCloud->push_back(cartesianPoint);
                        curFrameCloud->push_back(cartesianPoint);
                        // std::cout << "push\n";
                        
                        pointsConverted++;
                    }
                }
                // pointsFile << "\n";
            }

            frameClouds.push_back(curFrameCloud);
        }
        // pointsFile.close(); // was used for debugging - remove later if not needed
        std::cout << "frameClouds count: " << frameClouds.size() << "\n";

        std::cout << "tooSmallCount: " << tooSmallCount << "\n";

        auto t2 = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::chrono::duration<double, std::milli> ms_double = t2 - t1;
        std::cout << "duration: " << dur.count() << "ms\n";
        std::cout << "duration (d): "  << ms_double.count() << "ms\n";
        std::cout << "packet count: " <<  packets.size() << "\n"; 
        std::cout << "frameDataBlocks: " << frameDataBlocks.size() << "\n";
        std::cout << "points converted: " << pointsConverted << "\n";

        // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL Visualiser"));
        // viewer->setBackgroundColor(0,0,0);
        // std::cout << "Frame 1 Points: " << frameClouds[3]->points.size() << "\n";
        // viewer->addPointCloud<pcl::PointXYZI>(frameClouds[3], "Frame 1");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Frame 1");
        // viewer->addCoordinateSystem(1.0);
        // viewer->initCameraParameters();
        // viewer->setCameraPosition(0,16,0,0,0,1);

        // int frameCounter = 1;
        // auto lastTime = std::chrono::high_resolution_clock::now();
        // while(!viewer->wasStopped()) {
        //     viewer->spinOnce(100);
        //     std::cout << "frameNumber: " << frameCounter << "\n";
        //     if(frameCounter < frameClouds.size() && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastTime).count() > 100) {
        //         if(frameClouds[frameCounter]->points.size() > 24000) {
        //             viewer->removeAllPointClouds();
        //             viewer->removeAllShapes();
        //             // viewer->updatePointCloud(frameClouds[frameCounter], "Frame 1");
        //             std::string frameName = "Frame " + std::to_string(frameCounter);
                    
        //             pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudEdge(new pcl::PointCloud<pcl::PointXYZI>()); // JUST FOR DEBUGGING - REMOVE LATER
        //             pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudSurf(new pcl::PointCloud<pcl::PointXYZI>()); // JUST FOR DEBUGGING - REMOVE LATER
        //             pcl::PointCloud<pcl::PointXYZI>::Ptr pcFilter(new pcl::PointCloud<pcl::PointXYZI>()); // JUST FOR DEBUGGING - REMOVE LATER

                    
        //             laserProcessing.featureExtraction(frameClouds[frameCounter], pointCloudEdge, pointCloudSurf); // JUST FOR DEBUGGING - REMOVE LATER
                    
        //             Eigen::Vector4f minVec = Eigen::Vector4f(-10, -6.2, -2, 1);
        //             Eigen::Vector4f maxVec = Eigen::Vector4f(15, 7, 10, 1);

        //             pcFilter = objProcessor.filterCloud(frameClouds[frameCounter], 0.25, minVec, maxVec); // JUST FOR DEBUGGING - REMOVE LATER

        //             std::unordered_set<int> inliers = ransacPlane(pcFilter, 10, 0.4); // JUST FOR DEBUGGING - REMOVE LATER

        //             pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudInliers(new pcl::PointCloud<pcl::PointXYZI>()); // JUST FOR DEBUGGING - REMOVE LATER
        //             pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudOutliers(new pcl::PointCloud<pcl::PointXYZI>()); // JUST FOR DEBUGGING - REMOVE LATER

        //             for(int index = 0; index < pcFilter->points.size(); index++) { // JUST FOR DEBUGGING - REMOVE LATER
        //                 pcl::PointXYZI point = pcFilter->points[index];
        //                 if(inliers.count(index)) {
        //                     pointCloudInliers->points.push_back(point);
        //                 } else {
        //                     pointCloudOutliers->points.push_back(point);
        //                 }
        //             }

        //             renderPointCloud(viewer, pointCloudInliers, "Inliers", Colour(0,1,0));
        //             renderPointCloud(viewer, pointCloudOutliers, "Outliers", Colour(1,0,0.5));

        //             KdTree* tree = new KdTree;
        //             std::vector<std::vector<float>> pointVectors;

        //             for(int i = 0; i < pointCloudOutliers->points.size(); i++) {
        //                 std::vector<float> pointVector;
        //                 pointVector.push_back(pointCloudOutliers->points[i].x);
        //                 pointVector.push_back(pointCloudOutliers->points[i].y);
        //                 pointVector.push_back(pointCloudOutliers->points[i].z);
        //                 pointVectors.push_back(pointVector);
        //                 tree->insert(pointVector, i);
        //             }

        //             std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = euclideanCluster(pointVectors, tree, 0.25, 10);

        //             // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedPlanes = objProcessor.segmentPlane(frameClouds[frameCounter], 100, 0.3);

        //             // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> surfColourHandler(pointCloudSurf, 0, 255, 0);
        //             // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> edgeColourHandler(pointCloudEdge, 255, 0, 0);


        //             // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> segment1ColourHandler(segmentedPlanes.first, 0, 255, 0);
        //             // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> segment2ColourHandler(segmentedPlanes.second, 255, 0, 0);


        //             viewer->updatePointCloud<pcl::PointXYZI>(frameClouds[frameCounter], frameName);
        //             // viewer->removeAllPointClouds();
        //             // viewer->addPointCloud<pcl::PointXYZI>(frameClouds[frameCounter], frameName);
        //             // viewer->addPointCloud<pcl::PointXYZI>(pointCloudSurf, surfColourHandler, "Surf " + std::to_string(frameCounter));
        //             // viewer->addPointCloud<pcl::PointXYZI>(pointCloudEdge, edgeColourHandler, "Edge " + std::to_string(frameCounter));

        //             // For testing segmentation for obj detection
        //             // viewer->addPointCloud<pcl::PointXYZI>(segmentedPlanes.first, segment1ColourHandler, "Segment 1 " + std::to_string(frameCounter));
        //             // viewer->addPointCloud<pcl::PointXYZI>(segmentedPlanes.second, segment2ColourHandler, "Segment 2 " + std::to_string(frameCounter));
                    
        //             // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = objProcessor.clusterCloud(segmentedPlanes.first, 0.5, 3, 50);
        //             int clusterID = 1;
        //             for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters) {
        //                 // std::cout << "cluster size: " << cluster->points.size() << "\n";
        //                 renderPointCloud(viewer, cluster, "Cluster " + std::to_string(clusterID), Colour(0,0,1));

        //                 Box box = objProcessor.boundingBox(cluster);
        //                 renderBox(viewer, box, clusterID);
        //                 clusterID++;
        //             }

        //             frameCounter++;
        //         } else frameCounter++;
               
        //     }
        // }

        delete[] buffer;

    }
