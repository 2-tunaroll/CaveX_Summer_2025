#pragma once
#include <vector>
#include <iterator>
#include <iostream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <chrono>
#include <cmath>
#include "lidar/velodyneSocketReader.h"

namespace arachnida {
    double getLaserAngleFromChannelID(int channelID);

    double getLaserChannelTiming(
        int channelID, 
        int dataBlockID
    );

    // Parses packet assuming Single Return mode
    void parsePacketToPointCloud(
        std::vector<char> const &packet, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud
    );

    // Parses packet assuming Single Return mode
    // void parsePacketToDataBlocks(char *packet, std::vector<sock_velodyneVLP16DataBlock> &dataBlocks);
    void parsePacketToDataBlocks(
        std::vector<char> const &packet, 
        std::vector<arachnida::sock_velodyneVLP16DataBlock> &dataBlocks
    );

    // Parses frame assuming Single Return mode
    void parseFrameToPointCloud(
        std::vector<char> &frame, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud
    );
}
