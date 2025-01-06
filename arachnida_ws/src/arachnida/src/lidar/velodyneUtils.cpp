// This file contains a variety of useful functions
// for parsing LiDAR data
// To understand what is happening you'll need to
// read the relevant part of the VLP-16 user manual.
// This primarily includes chapters 8 and 9
#include "lidar/velodyneUtils.h"

#include <string>
#include <iomanip>
#include <unistd.h>
#include <pcl/PCLPointCloud2.h>

// Stores laser angles in radians for VLP16
// - Stores them in order of channel ID (i.e first element is the angle 
//   for channel 0, second element is the angle for channel 1, etc.)
// - Stores them twice for performance improvement in retrieval
// -- Each data block contains two firings from each laser and thus the 
//    channel ID can go up to 32 rather than 16
// -- Storing them twice means we don't have to subtract 16 from the 
//    channel ID when it is > 15
double laserChannelAngles[32] = { -0.261799,  0.0174533, -0.226893, 0.0523599, -0.191986, 0.0872665, -0.15708,  0.122173, 
                                 -0.122173,  0.15708,   -0.0872665, 0.191986, -0.0523599, 0.226893, -0.0174533, 0.261799,
                                 -0.261799,  0.0174533, -0.226893, 0.0523599, -0.191986, 0.0872665, -0.15708,  0.122173, 
                                 -0.122173,  0.15708,   -0.0872665, 0.191986, -0.0523599, 0.226893, -0.0174533, 0.261799 }; 

// Returns the vertical angle of a laser relative to the VLP16's horizon in 
// radians from a laser ID/channel ID
// param: channelID - the ID of the laser channel (as per VLP-16 manual) to
//                    find the timing for
double arachnida::getLaserAngleFromChannelID(int channelID) {
    if(channelID < 1 || channelID > 32) return 0; // return 0 if the channel ID is invalid
    else return laserChannelAngles[channelID - 1];
}

// Returns the delay in seconds from the first laser firing to the specified laser firing
// - The total time for all 16 lasers for fire is 55.296us
// - The time between each laser firing is 2.304us
// - There is an 18.432us idle time after the last laser firing in a sequence (happens twice per data block)
// param: channelID - the ID of the laser channel (as per VLP-16 manual) to find 
//                    the timing for
// param: dataBlockID - the ID of the data block (as per VLP-16 manual) to find
//                      the timing for
// Note: This function is not currently used. The timing should be used to
//       compute the azimuth of each individual point. However, currently,
//       the azimuth is retrieved from the packets for each data block
//       and this value is used for all the points in a data block.
//       This introduces a small amount of error but saves a significant
//       amount of computational power.
double arachnida::getLaserChannelTiming(int channelID, int dataBlockID) {
    if(channelID < 1 || channelID > 32) return 0; // return 0 if the channel ID is invalid
    if(dataBlockID < 1 || dataBlockID > 12) return 0; // return 0 if the data block ID is invalid
    else return (55.296*1e-6*dataBlockID) + (2.304*1e-6*channelID);
}

// This function takes a packet buffer and populates a point cloud with 
// the points from the packet in the form of a pcl PointCloud with 
// pcl PointXYZI points
// param: &packet - reference to the buffer containing the packet's raw binary data
// param: &pointCloud - reference to the point cloud to be populated from the packet buffer
void arachnida::parsePacketToPointCloud(std::vector<char> const &packet, pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud) {
    std::vector<arachnida::sock_velodyneVLP16DataBlock> dataBlocks;
    parsePacketToDataBlocks(packet, dataBlocks); // Populate dataBlocks with parsed LiDAR points from the packet

    for(int i = 0; i < dataBlocks.size(); i++) { // loop through each data block in the packet
        sock_velodyneVLP16DataBlock curDataBlock = dataBlocks[i];
        for(int j = 0; j < curDataBlock.points.size(); j++) { // loop through each point in the data block
            sock_velodyneVLP16Point curPoint = curDataBlock.points[j];
            pcl::PointXYZI pclPoint;
            pclPoint.x = curPoint.distance * cos(getLaserAngleFromChannelID(curPoint.channel)) * sin(curDataBlock.azimuth * M_PI / 180);
            pclPoint.y = curPoint.distance * cos(getLaserAngleFromChannelID(curPoint.channel)) * cos(curDataBlock.azimuth * M_PI / 180);
            pclPoint.z = curPoint.distance * sin(getLaserAngleFromChannelID(curPoint.channel));
            pclPoint.intensity = curPoint.reflectivity;
            pointCloud->points.push_back(pclPoint);
        }
    }
}

// This function takes a packet buffer and populates a vector of data blocks with
// the data blocks from the packet
// param: &packet - reference to the buffer containing the packet's raw binary data
// param: &dataBlocks - reference to the vector of data blocks to be populated from the packet buffer (*packet)
// void parsePacketToDataBlocks(char *packet, std::vector<sock_velodyneVLP16DataBlock> &dataBlocks) {
void arachnida::parsePacketToDataBlocks(std::vector<char> const &packet, std::vector<arachnida::sock_velodyneVLP16DataBlock> &dataBlocks) {
    // 1. Find start of first packet (0xFFEE flag)
    // 2. Find azimuth bytes (2 bytes)
    // 3. Convert azimuth bytes to azimuth angle (float)
    // 4. Find distance bytes (2 bytes) and reflectivity byte (1 byte)
    // 5. Convert distance bytes to distance (float) and reflectivity byte to reflectivity (float)
    // 6. Add point to data block
    // 7. Add data block to &dataBlocks

    bool ffFlag = false;
    unsigned int firstPacketInBlockTimestamp = 0;
    size_t packetSize = packet.size();

    if(packetSize != 1206) return; // return if the packet is too small (i.e. not at least one full packet minus the 42 byte UDP header)
    
    size_t numberOfPackets = floor(packetSize / 1206); // number of packets in the buffer
    if(numberOfPackets > 1) return; // return if there is more than one packet in the buffer

    for(int byteIndex = 0; byteIndex < packetSize; byteIndex++) {
        if(packet[byteIndex] == '\xFF') {
            ffFlag = true; // Found start of packet
        }
        else if(packet[byteIndex] == '\xEE') {
            if(ffFlag) { // if ffFlag is true then we must be at the start of a datablock (0xFFEE)
                ffFlag = false;

                unsigned int timestamp = ((unsigned int) packet[byteIndex+1201] << 24) | ((unsigned int) (packet[byteIndex+1200] << 16)) | ((unsigned int) (packet[byteIndex+1199] << 8)) | ((unsigned int) packet[byteIndex+1198]); // Gets the timestamp from the packet
                arachnida::sock_velodyneVLP16Packet curPacket; // The data block to be populated from the packet buffer (*packet) and added to the packet

                for(int datablock = 0; datablock < 12; datablock++) { // loop through each data block in the packet
                    if(datablock > 0) byteIndex += 4; // add 4 to byteIndex to get to the 0xEE byte (adding 3 to get from first dist byte of last channel to 0xFF byte of 0xFFEE bytes, then add another 1 to get to the 0xEE byte)
                    unsigned int blockTimestamp = timestamp + datablock*55.296*2; // Stores the time of the first laser firing in a datablock; 55.296us per firing sequence and two firing sequences per data block 
                    arachnida::sock_velodyneVLP16DataBlock curDataBlock;
                    curDataBlock.azimuth = ((float)((unsigned char) packet[byteIndex+2] << 8 | (unsigned char) packet[byteIndex+1])) / 100; // byteIndex+2 and byteIndex+1 are the two bytes which comprise the azimuth (see VLP16 manual)
                    
                    for(int channel = 0; channel < 32; channel++) {
                        byteIndex += 3; // Sets byteIndex to the first byte of the channel+1th channel (e.g if channel is 0, i is the first byte of channel 1)
                        arachnida::sock_velodyneVLP16Point curPoint;
                        curPoint.channel = channel + 1;
                        curPoint.distance = ((float)(((unsigned int) packet[byteIndex+1]) << 8 | ((unsigned int) packet[byteIndex]))) / 500; // divide by 500 to get distance in m (see VLP16 manual)
                        curPoint.reflectivity = (float) packet[byteIndex+2];
                        curDataBlock.points.push_back(curPoint); // Adds the point to the data block 
                    }
                    dataBlocks.push_back(curDataBlock); // Adds the data block to the vector of data blocks
                }
                byteIndex += 4; // Add 4 to the byteIndex to get the byte before the last set of distance + reflectivity bytes to the first timestamp byte
                // }
            }
        }
    }
}

// This function takes a frame buffer and populates a point cloud with the points 
// from the frame in the form of a pcl PointCloud with pcl PointXYZI points
// param: &frame - reference to the buffer containing the frame's raw binary data
// param: &pointCloud - reference to the point cloud to be populated from the frame buffer
void arachnida::parseFrameToPointCloud(std::vector<char> &frame, pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud) {
    std::vector<char> curPacket; // stores the bytes for the current packet being isolated
    int packetIndexTracker = 0;
    for(int i = 0; i < frame.size()-1; i++) {
        if(i == frame.size()-1) { // if we are at the last byte in the frame
            curPacket.push_back(frame[i]);
            arachnida::parsePacketToPointCloud(curPacket, pointCloud);
            curPacket.clear();
            break;
        }
        if(packetIndexTracker < 1206) {
            packetIndexTracker++;
            curPacket.push_back(frame[i]);
        } else {
            packetIndexTracker = 0;
            arachnida::parsePacketToPointCloud(curPacket, pointCloud);
            curPacket.clear();
        }
    }
}