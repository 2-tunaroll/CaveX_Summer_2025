// Sample publisher node.
// Doesn't do anything meaningful; simply
// publishes a string in a loop
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "publisher_sample");
    ros::NodeHandle nh;

    ros::Publisher publisher_pub = nh.advertise<std_msgs::String>("/publisher_pub", 1000);
    ros::Rate loop_rate(10);
    int count = 0;

    while(ros::ok()) {
        std_msgs::String rosMsg;
        std::stringstream ss;
        
        ss << "Hello ROS World [" << count << "]";
        rosMsg.data = ss.str();

        ROS_INFO("%s", rosMsg.data.c_str());
        publisher_pub.publish(rosMsg);

        ros::spinOnce();
        loop_rate.sleep();

        ++count;
    }

    return 0;
}
