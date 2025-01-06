// This file is used to connect with the VLP-16 via a UDP socket
#pragma once
#include <fstream>
#include <vector>
#include <iterator>
#include <iostream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>
#include "ros/ros.h"

// One frame is the data from one full rotation of the VLP-16
#define FRAME_SIZE_BYTES 94037 // approx. 94037 bytes per frame (VLP-16 manual reports data rate of 940368 bytes/sec)
#define MAX_FRAME_BUFFER_QUEUE_SIZE_BYTES 4701840 // approx. 5 seconds worth of data (940368 bytes/sec * 5 sec)

namespace arachnida {
    // 3 out of 4 pieces of the data required to 
    // describe a single point detected. The
    // missing piece, the azimuth, is stored
    // on at each data block (see 
    // sock_velodyneVLP16DataBlock for details).
    // distance: distance in metres from LiDAR
    //           detected point
    // reflectivity: the reflectivitiy of a 
    //               point as per VLP-16
    //               manual.
    // channel: the laser channel, as per VLP-16
    //          manual, which maps to a specific 
    //          laser and thus a vertical angle
    //          relative to the LiDAR's 
    //          "horizontal" plane
    struct sock_velodyneVLP16Point {
        float distance;
        float reflectivity;
        int channel;
    };

    // A data block as described in the VLP-16 manual
    // with the caveat that azimuth is treated as identical
    // for all points. As the lasers fire ~55us apart, the azimuth
    // is not truly the same. However, we are accepting this
    // tiny amount of error on the basis it has little impact
    // on accuracy of results and that computing the azimuth
    // for each point individually is unnecessarily
    // computationally expensive.
    // azimuth: the rotational angle of the VLP-16 about it's
    //          "vertical" axis
    // points: the points within the data block
    struct sock_velodyneVLP16DataBlock {
        float azimuth;
        std::vector<sock_velodyneVLP16Point> points; // array store of points (distances) for VLP-16
    };  

    // Stores all the data blocks (and thus the points) 
    // from a particular packet within their associated 
    // data blocks. 
    // timestamp: the timestamp of the packet as defined in
    //            VLP-16 manual
    // dualReturnMode: whether or not the VLP-16 is in dual
    //                 return mode as per the VLP-16 manual
    // dataBlocks: the data blocks (and thus the points) 
    //             contained in the packet
    struct sock_velodyneVLP16Packet {
        unsigned int timestamp;
        bool dualReturnMode = false;
        std::vector<sock_velodyneVLP16DataBlock> dataBlocks;
    };

    // Stores all the packets for a particular frame (one
    // full rotation of the VLP-16's laser array)
    // packets: all the packets for the frame
    struct sock_velodyneVLP16Frame {
        std::vector<sock_velodyneVLP16Packet> packets;
    };

    // Stores the data blocks (sock_velodyneVLP16DataBlocks) 
    // for a single frame
    // 1 frame = points from one full rotation of the VLP-16's 
    // laser array
    struct sock_velodyneVLP16FrameDataBlocks {
        std::vector<sock_velodyneVLP16DataBlock> dataBlocks; // all the dataBlocks in a frame
    };

    class velodyneSocketReader {
        private:
            int PORT = 2368;
            int sockfd;
            int socketID;
            int valRead;
            sockaddr_in address;
            int opt = 1;
            int addrlen = sizeof(address);
            char buffer[1206] = { 0 }; // deprecated (superceded by packetBuffer)
            int laserAngles[16] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15}; // laser angles in deg (index of entry is the laser ID) // deprecated (superceded by velodyneUtils.cpp)
            std::chrono::time_point<std::chrono::system_clock> lastPacketTimestamp;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;
        public:
            velodyneSocketReader();
            int getPort() { return PORT; }
            int getSockFileDescriptor() { return sockfd; }
            int getSocketID() { return socketID; }
            int getValRead() { return valRead; }
            sockaddr_in getAddress() { return address; }
            int getOpt() { return opt; }

            // Connects to the LiDAR via a socket listening to 
            // the specified port and stores the data in the 
            // packetBuffer until a full frame is received.
            // A full frame is 100ms of data, which is roughly 
            // 94037 bytes of data (VLP-16 manual reports data 
            // rate of 940368 bytes/sec)
            // @param frameBuffer: stores the raw binary data from 
            // the lidar until a full frame is received.
            // @param frameBufferQueue: stores yet-to-be-processed
            // lidar frames as needed
            // @param lidarPub: the ROS publisher object 
            // the lidar data should be published to.
            // @param writeToFile: true if the lidar 
            // data should be written to a file, false
            // if not. NOTE: The file contains only 
            // the LiDAR portion of the whole UDP packet.
            // Existing software for viewing the data as
            // a point cloud (e.g VeloView) needs the 
            // UDP headers to be in the file to render
            // the data. Hence, to use the data, you'll
            // need to write a script which appends dummy
            // UDP headers to each packet in the file.
            void connect(
                std::vector<char> &frameBuffer, 
                std::vector<std::vector<char>> &frameBufferQueue, 
                ros::Publisher &lidarPub, 
                bool writeToFile
            );
            
            // Close the socket listening to the VLP-16 and
            // print a message to indicate succesful closure.
            void disconnect();

            // static std::vector<char> packetBuffer; // stores the raw binary data from the lidar
            static std::vector<sock_velodyneVLP16Packet> packets;
            static std::vector<sock_velodyneVLP16Frame> frames;
            static std::vector<sock_velodyneVLP16FrameDataBlocks> frameDataBlocks;
            static std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> frameClouds; // vector of point clouds; one point cloud per frame (this is what would be passed to SLAM)
    };
}