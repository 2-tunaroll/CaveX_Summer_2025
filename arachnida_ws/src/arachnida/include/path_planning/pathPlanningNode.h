#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "path_planning/artificialPotentialField.h"

#include <eigen3/Eigen/Dense>

namespace arachnida {
    class pathPlanner {
        public:
            arachnida::path_planning::artificialPotentialField APF;
            pathPlanner(void);
            void pathPlannerInitialiser(void);
            void currentPositionCallback(const nav_msgs::Odometry &currentPoseMsg);
            //void cumulativePositionCallback(const geometry_msgs::PoseStamped &cumulativePoseMsg);
            void objectsDetectedCallback(const arachnida::ObstacleList &obstacleList);
            void planPath(void);
            void publishDirection(Eigen::Vector3d direction);

        private:
            // subscriber to current position of the robot from SLAM
            ros::Subscriber current_position_sub_;

            // subscriber to cumulative position of the robot from SLAM
            ros::Subscriber cumulative_position_sub_;

            // subscriber to list of obstacles
            ros::Subscriber obstacle_list_sub_;

            // publisher to topic /syropod_remote/desired_velocity
            ros::Publisher desired_velocity_pub_;

            // message published to topic /syropod_remote/desired_velocity
            geometry_msgs::Twist desired_velocity_msg_;

            // desired direction vector
            Eigen::Vector3d desiredDirection;
    };
}
