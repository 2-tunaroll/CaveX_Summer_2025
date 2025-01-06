#include "gait_energistics/gait_energetics.h"

GaitEnergetics::GaitEnergetics(){
    // constructor
}

std_msgs::Int8 GaitEnergetics::getGaitType(){
    return gaitType;
}

void GaitEnergetics::checkPowerVector(){
    // A helper function to ensure only the last 500 samples are recorded
    if (powerConsumptionData.size() > 500){
        powerConsumptionData.erase(powerConsumptionData.begin()); // remove oldest element which is at the front of the vector
    }
}

float GaitEnergetics::powerAverage(float duration){
    // Helper function to work out the power for the last duration seconds
    int numData = duration * msgFrequency; // cast as integer
    // get position of most recent power consumption
    int endPosition = powerConsumptionData.size() - 1;
    float powerSum = 0; // store sum of power data
    for (int i = endPosition; i > endPosition - numData; i--){
        // start at endPosition and read required number of data
        powerSum += powerConsumptionData.at(i);
    }
    // return average power
    return powerSum / numData;
}

void GaitEnergetics::setGait(ros::Time timeStamp){
    // std::cout << "TIME SINCE LAST GAIT SWITCH: " << elapsedTime << std::endl;
    if (elapsedTime > 2*longTime){
        // used to allow for the power consumption data to stabilise after a gait switch
        float power = powerAverage(shortTime);
        float avgPower = powerAverage(longTime);
        //std::cout << "SHORT TERM POWER AVG: " << std::endl;
        //std::cout << power << std::endl; // print short term average
        //std::cout << "LONG TERM POWER AVG: " << std::endl;
        //std::cout << avgPower << std::endl; // print long term average

        if (gaitType.data == gaits.tripod){
            // currently tripod gait
            if (fabs(avgPower - power) > tripodPowerTolerance){
                // set gait type to  wave
                elapsedTime = 0;
                previousTimeStamp = timeStamp;
                gaitType.data = gaits.wave;
                newGait = true;
            }
        }
        else if (gaitType.data == gaits.wave){
            // currently in wave gait
            if (fabs(avgPower - power) > wavePowerTolerance){
                // set gait type to tripod
                elapsedTime = 0;
                previousTimeStamp = timeStamp;
                gaitType.data = gaits.tripod;
                newGait = true;
            }
        }
    }
}

void GaitEnergetics::readJointEfforts(const sensor_msgs::JointState msg){
    // log messages we are subscribing to
    // each packet has data for 18 motors
    ros::Time timeStamp = msg.header.stamp; // get current rostime
    // if first data packet mark this as the first timestamp
    if (initalisation){
        previousTimeStamp = timeStamp;
        initalisation = false;
    }

    float currentConsumption = 0;
    float powerConsumption = 0;
    // udpate elapsed time using time stamp data
    elapsedTime = timeStamp.toSec() - previousTimeStamp.toSec();
    //std::cout << "ELAPSED TIME: " << std::endl;
    //std::cout << elapsedTime << std::endl;
    //std::cout << "MOTOR CURRENTS: " << std::endl;

    for (int i = 0; i < msg.effort.size(); i++){
        //std::cout << msg.effort.at(i) << std::endl; // print individual joint effort
        currentConsumption += fabs(msg.effort.at(i));
    }
    powerConsumption = currentConsumption * 12; // multiply by operating voltage
    // add powerConsumption to vector
    powerConsumptionData.push_back(powerConsumption);
    checkPowerVector(); // check length does not exceed 500
    
    //std::cout << "Total Motor Power Consumption:" << std::endl;
    //std::cout << powerConsumption << std::endl;
    setGait(timeStamp);
}
