// This class is used to load .pcap files
// .pcap files are "packet capture" files
// which store raw network data (e.g
// VLP-16 packets) and can be recorded 
// using WireShark
#pragma once
#include <fstream>
#include <vector>
#include <iterator>
#include <iostream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace arachnida {
    // 3 out of 4 pieces of the data required to 
    // describe a single point detected. The
    // missing piece, the azimuth, is stored
    // on at each data block (see 
    // velodyneVLP16DataBlock for details).
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
    struct velodyneVLP16Point {
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
    struct velodyneVLP16DataBlock {
        float azimuth;
        std::vector<velodyneVLP16Point> points; // array store of points (distances) for VLP-16
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
    struct velodyneVLP16Packet {
        unsigned int timestamp;
        bool dualReturnMode = false;
        std::vector<velodyneVLP16DataBlock> dataBlocks;
    };

    // Stores all the packets for a particular frame (one
    // full rotation of the VLP-16's laser array)
    // packets: all the packets for the frame
    struct velodyneVLP16Frame {
        std::vector<velodyneVLP16Packet> packets;
    };

    // Stores the data blocks (velodyneVLP16DataBlocks) 
    // for a single frame
    // 1 frame = points from one full rotation of the VLP-16's 
    // laser array
    struct velodyneVLP16FrameDataBlocks {
        std::vector<velodyneVLP16DataBlock> dataBlocks; // all the dataBlocks in a frame
    };

    class velodynePCAPReader {    
        private:
            std::string absolutePath;
            std::vector<char> pcapBuffer;
            std::vector<velodyneVLP16Packet> packets;
            std::vector<velodyneVLP16Frame> frames;
            std::vector<velodyneVLP16FrameDataBlocks> frameDataBlocks;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> frameClouds; // vector of point clouds; one point cloud per frame (this is what would be passed to SLAM)
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;
            int laserAngles[16] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15}; // laser angles in deg (index of entry is the laser ID)       

        public:
            velodynePCAPReader(std::string absolutePath);
            
            std::string getAbsolutePath() { return absolutePath; }

            std::vector<char> getPCAPBuffer() { return pcapBuffer; }

            std::vector<velodyneVLP16Packet> getPackets() { return packets; }

            std::vector<velodyneVLP16Frame> getFrames() { return frames; }

            std::vector<velodyneVLP16FrameDataBlocks> getFrameDataBlocks() { return frameDataBlocks; }

            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> getFrameClouds() { return frameClouds; }

            pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud() { return pointCloud; }

            void readFile();
    };
}
