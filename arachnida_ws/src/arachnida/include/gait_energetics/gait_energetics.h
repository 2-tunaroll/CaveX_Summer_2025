#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include "sensor_msgs/JointState.h"
#include <vector>

#pragma once

struct gaitTypes {
    int wave = 0;
    int amble = 1;
    int ripple = 2;
    int tripod = 3;
};

class GaitEnergetics {
    public:
        // Data
        gaitTypes gaits; // used tp reference gait types
        std_msgs::Int8 gaitType; // used to set gait Type under gait_selection topic
        std::vector<float> powerConsumptionData; // used to store instantaneous motor power consumption
        float longTime = 3; // time for long term power average in (s)
        float shortTime = 0.5; // time for short term power average in (s)
        float tripodPowerTolerance = 6; // Maximum different in long-term and instantanoues power averages (W)
        float wavePowerTolerance = 8; // Maximum different in long-term and instantanoues power averages (W)
        float amblePowerTolerance = 6.5; // Maximum different in long-term and instantanoues power averages (W)
        float ripplePowerTolerance = 8.5; // Maximum different in long-term and instantanoues power averages (W)
        float msgFrequency = 60; // Hz. Frequency of motor data being published
        ros::Time previousTimeStamp; // used to store previous time stamp of data
        float elapsedTime = 0; // time since last gait switch initalised as 0s
        bool initalisation = true; // used to determine if the time data needs initalisation
        bool newGait = false; // used to tell publisher whether or not to publish new gait type

        // Methods
        GaitEnergetics(); // constructor
        void checkPowerVector(); // used to check size of power consumption vector
        float powerAverage(float duration); // used to average the power consumption data for duration given
        void setGait(ros::Time timeStamp); // used to decided whether a gait should be switched
        void readJointEfforts(const sensor_msgs::JointState msg);
        std_msgs::Int8 getGaitType();
};
