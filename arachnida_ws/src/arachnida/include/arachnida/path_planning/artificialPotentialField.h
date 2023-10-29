#pragma once

#include <cstdio>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include <string>

#include "../../arachnida/utils.h"

#include <eigen3/Eigen/Dense>

namespace arachnida {
    namespace path_planning {

        typedef struct Obstacle {
            int id;
            Eigen::Vector3d location;
            float radius;
            float gainRepulsiveForce = 0.05;
        } Obstacle;

        class artificialPotentialField{
            private:
                Eigen::Vector3d start; // start position of robot
                Eigen::Vector3d goal; // goal position of robot
                Eigen::Vector3d currPosition; // instantaneous position of the robot
                std::vector<Obstacle> obstacles; // vector of obstacles detected
                std::map<Eigen::Vector3d, double> distanceToGoal;
                float gainAttractiveForce = 1.0;
                float minimumRadius = 0.25; // Keep at least 0.25 m away from obstacle
                float stepSize = 0.01; // 1cm step size
            public:
                artificialPotentialField(Eigen::Vector3d start, Eigen::Vector3d goal, Eigen::Vector3d curPosition);
                double calculateDistanceToGoal(Eigen::Vector3d currPosition, Eigen::Vector3d goal);
                double calculateDistanceToObstacle(Eigen::Vector3d currPosition, Eigen::Vector3d obstacleLocation);
                double calculateGoalPotential(double distance);
                double calculateObstaclePotential(Obstacle obstacle);
                double calculateObstaclePotential(Eigen::Vector3d position, Obstacle obstacle);
                double calculateOverallLocalPotential(std::vector<Obstacle> obstacles);
                void updateRepulsiveGain(std::vector<Obstacle> obstacles);
                Eigen::Vector3d generateDirectionVector(Eigen::Vector3d currPosition);
                
        };
    }
}