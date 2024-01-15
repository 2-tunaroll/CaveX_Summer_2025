#define M_PI 3.14159265358979323846

#include <cstdio>
#include <pcl/conversions.h>
#include "arachnida/lidar/velodyneSocketReader.h"
#include "arachnida/lidar/velodyneUtils.h"
#include <cmath>
#include <errno.h>
#include <string>
#include <fcntl.h>
#include <sstream>
#include <poll.h>
#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include "floam_cpu/laserProcessingClass.h"
#include "object_detection_cpu/objPointCloudProcessor.h"
#include "object_detection_cpu/objRansac.h"
#include "object_detection_cpu/objCluster.h"
#include "object_detection_cpu/objRender.h"
#include "object_detection_cpu/objBox.h"
#include "object_detection_cpu/objKdtree.h"
#include "sensor_msgs/PointCloud2.h"
#include <fstream>

// Constructor to set up some of the initial values
// for opening a socket to the VLP-16
arachnida::velodyneSocketReader::velodyneSocketReader() {
    PORT = 2368;
    address.sin_family = AF_INET;
    memset(&address, 0, sizeof(address));
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);
};


int arachnidaLive_sockfd;

void arachnida::velodyneSocketReader::connect(std::vector<char> &frameBuffer, std::vector<std::vector<char>> &frameBufferQueue, ros::Publisher &lidarPub, bool writeToFile) {
    int frameCounter = 1;

    lastPacketTimestamp = std::chrono::high_resolution_clock::now();

    std::string fileName;
    std::FILE* file;
    for(int currentFileVersion = 1; currentFileVersion < 1000; currentFileVersion++) {
        std::string possibleFileName = "/home/cavex/Documents/velodyneSocketReader_LiDAR_Recording" + std::to_string(currentFileVersion) + ".pcap";
        file = std::fopen(possibleFileName.c_str(), "r");
        if(file == NULL) { // if file does not exist
            fileName = possibleFileName;
            break;
        } else continue;
    }

    std::ofstream rawDataFile(fileName);
    int rawDataFileCurrentSizeBytes = 0;
    int rawDataFileCurrentIteration = 0;

    sockfd = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        std::cout << "[velodyneSocketReader.cpp] Socket creation failed\n";
        exit(EXIT_FAILURE);
        return;
    }

    int val = 1;
    if(setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) == -1) {
        std::cout << "[velodyneSocketReader.cpp] setsockopt failed\n";
        perror("socketopt");
        exit(EXIT_FAILURE);
        return;
    };

    int bindStatus = bind(sockfd, (struct sockaddr *)&address, sizeof(address));

    if(bindStatus < 0) {
        std::cout << "[velodyneSocketReader.cpp] Bind failed\n";
        perror("bind");
        exit(EXIT_FAILURE);
        return;
    }

    if(fcntl(sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        std::cout << "[velodyneSocketReader.cpp] fcntl failed\n";
        perror("non-block");
        exit(EXIT_FAILURE);
        return;
    }

    std::cout << "[velodyneSocketReader.cpp] sockfd: " << sockfd << "\n";
    std::cout << "[velodyneSocketReader.cpp] bindStatus: " << bindStatus << "\n";

    // getPacket
    struct pollfd fds[1];
    fds[0].fd = sockfd;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000;

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    int packetCounter = 0;
    int arrayIndexTracker = 0; // counts from 0 to 94036 (94037 bytes of data in a frame)
    int frameBufferQueueArrayIndexTracker = 0; // Counts from 0 to 50 (50 frames in the queue) then gets reset to zero to start overwriting the oldest frame in the queue
    
    // START: SETUP PASS LIDAR DATA FROM PORT 2368 TO PORT 6000
    // This is optional as it is for passing LiDAR data to the
    // local node.js/expressjs server which subsequently passed the
    // data to a backend node.js/expressjs server and then to
    // the NextJS front end (see arachnida_web and
    // websocket_ws in the CaveX2023 git repo)
    char arachnidaLive_buffer[1206];
    struct sockaddr_in arachnidaLive_servaddr;
    
    if((arachnidaLive_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("[velodyneSocketReader.cpp] Arachnida Live Socket Creation Failed");
    }

    memset(&arachnidaLive_servaddr, 0, sizeof(arachnidaLive_servaddr));

    // Filling server information
    arachnidaLive_servaddr.sin_family = AF_INET;
    arachnidaLive_servaddr.sin_port = htons(6000);
    arachnidaLive_servaddr.sin_addr.s_addr = INADDR_ANY;

    int arachnidaLive_n;
    socklen_t arachnidaLive_len;
    // END: SETUP PASS LIDAR DATA FROM PORT 2368 TO PORT 6000

    lastPacketTimestamp = std::chrono::high_resolution_clock::now();
    auto lastObjectDetectionTimestamp = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the edge points from the frame
	
    while(true) {
        do {
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if(retval < 0) {
                if(errno != EINTR) {
                    std::cout << "[velodyneSocketReader.cpp] poll() error: " << strerror(errno) << "\n";
                }
                return;
            }
            else if(retval == 0) {
                std::cout << "[velodyneSocketReader.cpp] poll() timeout\n";
                return;
            }
            if((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) {
                std::cout << "[velodyneSocketReader.cpp] poll() reports Velodyne device error\n";
                return;
            }
        } while((fds[0].revents & POLLIN) == 0);

        ssize_t nbytes = recvfrom(sockfd, buffer, 1206, 0, (sockaddr*) &sender_address, &sender_address_len);

        if(nbytes < 0) {
            if(errno != EWOULDBLOCK) {
                std::cout << "[velodneSocketReader.cpp] recvfail: " << strerror(errno) << "\n";
                return;
            }
        } else if((size_t) nbytes == 1206) {
            packetCounter++;
            int arachnidaLive_result = sendto(arachnidaLive_sockfd, (const char *)buffer, nbytes, MSG_CONFIRM, (const struct sockaddr *)&arachnidaLive_servaddr, sizeof(arachnidaLive_servaddr));
            if(arachnidaLive_result < 0) {
                perror("Arachnida Live Failed to Send");
            }

            for (int i = 0; i < 1206; ++i) {
                if(frameBufferQueue.size() > 1000) frameBufferQueue.erase(frameBufferQueue.begin());
                frameBuffer.push_back(buffer[i]);
                if(writeToFile) { 
                    if(rawDataFileCurrentSizeBytes > 62914560) { // i.e if the file that is currently being written to is larger than 60MB
                        rawDataFile.close();
                        rawDataFile.clear();
                        std::string newFileIterationName =  fileName + " iteration " + std::to_string(rawDataFileCurrentIteration) + ".pcap";
                        rawDataFile.open(newFileIterationName);
                        rawDataFileCurrentIteration++;
                        rawDataFileCurrentSizeBytes = 0;
                    }
                    rawDataFile << buffer[i]; // Write bytes to file
                    rawDataFileCurrentSizeBytes++;
                }
            }
            if(frameBuffer.size() > 94036) {
                frameBufferQueue.push_back(frameBuffer);
                frameBufferQueueArrayIndexTracker++;
                parseFrameToPointCloud(frameBufferQueue.back(), pc);
                frameBuffer.clear();
            }

            if(pc->size() > 29000) {
                frameCounter++;

				pcl::PCLPointCloud2 cloud2;
				cloud2.header.seq = frameCounter;
				pcl::toPCLPointCloud2(*pc, cloud2);
				lidarPub.publish(cloud2);
            }
        } else {
            std::cout << "[velodyneSocketReader.cpp] Incomplete velodyne packet read: " << nbytes << " bytes\n";
        }
        if(pc->size() > 29000) pc->clear();
    } 

    // if(socketID < 0) {
    //     std::cout << "Accept failed\n";
    //     std::cout << "Err: " << socketID << "\n";
    //     std::cout << "errno: " << errno << "\n";
    //     std::cout << "errnostr: " << strerror(errno) << "\n";
    //     exit(EXIT_FAILURE);
    //     return;
    // }

    // valRead = read(socketID, buffer, 1248);
    // printf("%s\n", buffer);
    
    // REMOVE LATER
    close(arachnidaLive_sockfd);

    if(!writeToFile) std::remove(fileName.c_str()); // Remove pcap file if write to file wasn't set to true
    

}

void arachnida::velodyneSocketReader::disconnect() {
    close(sockfd);
    std::cout << "[velodyneSocketReader.cpp] Socket closed\n";
}
