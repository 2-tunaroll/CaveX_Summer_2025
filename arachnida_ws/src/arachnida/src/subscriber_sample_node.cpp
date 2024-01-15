// Sample subscriber node.
// Doesn't do anything meaningful; simply
// listens to the data from the sample 
// publisher node (publisher_sample_node)
#include "ros/ros.h"
#include "std_msgs/String.h"

void talkerCallback(const std_msgs::String::ConstPtr& rosMsg) {
    ROS_INFO("I heard: [%s]", rosMsg->data.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "subscriber_sample");
    ros::NodeHandle nh;

    ros::Subscriber talker_sub = nh.subscribe("/publisher_sample", 1000, talkerCallback);
    ros::spin();
}