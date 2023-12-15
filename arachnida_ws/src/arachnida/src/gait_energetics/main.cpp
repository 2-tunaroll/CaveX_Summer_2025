//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Written by Arachnida 2023 team, gait energetics code allows the robot to relate changes in power consumption
// to traction losses and maintains traversal efficiency by switching the more effective gaits.
// The algorithm is based on 'Energetics-Informed Hexapod Gait Transitions Across Terrains' by Kottege et al. 2015
#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "gait_energetics.h"
#include <ostream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <string>
#include <unistd.h>

// define global variables
// create instance of object used for monitoring power consumptions and calculating averages
GaitEnergetics* gaitEnergeticsTool = new GaitEnergetics();
std::string controlMethod; // store the current control method from ros parameter
geometry_msgs::Twist previous_velocity; // store previous command to maintain traversal  direction after switch

void jointStatesCallback(const sensor_msgs::JointState msg){
    // joint state subscriber callback function
    if (controlMethod != "joy"){
        // only operate if not in joy mode to prevent conflicting messages
        gaitEnergeticsTool->readJointEfforts(msg);
    }
}

void updateVelocity(const geometry_msgs::Twist msg){
    // callback to update current velocity commands
	previous_velocity = msg;
}

int main(int argc, char **argv){
    gaitTypes gaits; // defined in gait energetics header file
    // initalise ros publisher and subscriber node
    ros::init(argc, argv, "GaitEnergeticsNode");
    ros::NodeHandle gaitEnergeticsNode;

    // get current control method parameter
    gaitEnergeticsNode.getParam("/syropod_remote/control_method",controlMethod);

    // intialise ros publisher and subscriber
    ros::Publisher gait_selection = gaitEnergeticsNode.advertise<std_msgs::Int8>("syropod_remote/gait_selection", 1);
    ros::Publisher velocity_command = gaitEnergeticsNode.advertise<geometry_msgs::Twist>("syropod_remote/desired_velocity", 1);
    ros::Subscriber joint_efforts = gaitEnergeticsNode.subscribe("joint_states",1000,jointStatesCallback);
    ros::Subscriber previous_velocity_command = gaitEnergeticsNode.subscribe("syropod_remote/desired_velocity",1000,updateVelocity);
    
    // intialise the system by ensuring the default gait is tripod (SHC does this too)
    gaitEnergeticsTool->gaitType.data = gaits.tripod;
    gait_selection.publish(gaitEnergeticsTool->gaitType);

    ros::Rate rate(10);

    while (ros::ok()){
        ros::spinOnce();
        // get current control method parameter (may change during ros runtime)
        gaitEnergeticsNode.getParam("/syropod_remote/control_method",controlMethod);

        if (gaitEnergeticsTool->newGait && controlMethod != "joy"){
		    std::cout << "Here!" << std::endl;
            gait_selection.publish(gaitEnergeticsTool->getGaitType());
            gaitEnergeticsTool->newGait = false;

            // check which gait was selected
	        if (gaitEnergeticsTool->getGaitType().data == gaits.wave){
		        sleep(6); // wave takes longer to switch sleep until switch is done
	        }else if (gaitEnergeticsTool->getGaitType().data == gaits.tripod){
		        sleep(4); // tripod is quicker to switch
	        }
            // maintain previous direction
	        velocity_command.publish(previous_velocity);
        }
        rate.sleep();
    }
    delete gaitEnergeticsTool;
    return 0;
}
