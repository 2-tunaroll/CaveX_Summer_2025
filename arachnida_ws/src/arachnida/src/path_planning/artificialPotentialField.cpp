#include "path_planning/artificialPotentialField.h"

arachnida::path_planning::artificialPotentialField::artificialPotentialField() {
    // default constructor
}

// Euclidean distance to goal position from current position
double arachnida::path_planning::artificialPotentialField::calculateDistanceToGoal(Eigen::Vector3d position, Eigen::Vector3d goal) {
    return sqrt(pow(goal.x() - position.x(), 2) + pow(goal.y() - position.y(), 2) + pow(goal.z() - position.z(), 2));
}

// Euclidean distance to an obstacle
double arachnida::path_planning::artificialPotentialField::calculateDistanceToObstacle(Eigen::Vector3d position, Eigen::Vector3d obstacleLocation) {
    return sqrt(pow(obstacleLocation.x() - position.x(), 2) + pow(obstacleLocation.y() - position.y(), 2) + pow(obstacleLocation.z() - position.z(), 2));
}

// Calculate attractive goal potential
double arachnida::path_planning::artificialPotentialField::calculateGoalPotential(double distance) {
    double goalDistance = calculateDistanceToGoal(currPosition, goal);
    return gainAttractiveForce / goalDistance;
}

// Calculate repulsive obstacle potential given an obstacle
double arachnida::path_planning::artificialPotentialField::calculateObstaclePotential(Obstacle obstacle) {
    double obstacleDistance = calculateDistanceToObstacle(currPosition, obstacle.location);
    double safeDistance = obstacleDistance - minimumRadius;
    return obstacle.gainRepulsiveForce / safeDistance;
}

// calculate repulsive obstacle potential given a position in the field and an obstacle
double arachnida::path_planning::artificialPotentialField::calculateObstaclePotential(Eigen::Vector3d position, Obstacle obstacle) {
    double obstacleDistance = calculateDistanceToObstacle(position, obstacle.location);
    double safeDistance = obstacleDistance - minimumRadius;
    if (safeDistance <= 0) {
        std::cout << "No Solution Exists, Setting New Waypoint" << std::endl;
        setGoal();
    }
    return obstacle.gainRepulsiveForce / safeDistance;
}

// calcualte overall local potential from goal and obstacles
double arachnida::path_planning::artificialPotentialField::calculateOverallLocalPotential(std::vector<Obstacle> obstacles) {
    double localPotential = 0.0;
    double goalDistance = calculateDistanceToGoal(currPosition,goal);
    
    for (int i = 0; i < static_cast<int>(obstacles.size()) - 1; i++) {
        localPotential += calculateObstaclePotential(obstacles.at(i));
    }
    
    return calculateGoalPotential(goalDistance) + localPotential;
}

// update the repulsive gain (as a function of distance)
void arachnida::path_planning::artificialPotentialField::updateRepulsiveGain(std::vector<Obstacle> obstacles) {
    for (int i = 0; i < static_cast<int>(obstacles.size()) - 1; i++) {
        double distance = calculateDistanceToObstacle(currPosition,obstacles.at(i).location);
        obstacles.at(i).gainRepulsiveForce -= exp(-2*distance);

        if (obstacles.at(i).gainRepulsiveForce > 1) {
            obstacles.at(i).gainRepulsiveForce = 1;
        } else if (obstacles.at(i).gainRepulsiveForce < 0.05) {
            obstacles.at(i).gainRepulsiveForce = 0.05;
        }
    }
}

// set the obstacle list given obstacle data
void arachnida::path_planning::artificialPotentialField::setObstacleList(const arachnida::ObstacleList &obstacleList) {
    std::vector<Obstacle> objects;

    for (int i = 0; i < static_cast<int>(obstacleList.obstacles.size()) - 1; i++) {
        Obstacle obj;
        obj.id = obstacleList.obstacles.at(i).id;
        Eigen::Vector3d location(obstacleList.obstacles.at(i).x,obstacleList.obstacles.at(i).y,obstacleList.obstacles.at(i).z);
        obj.location = location;
        obj.radius = obstacleList.obstacles.at(i).radius;
        objects.push_back(obj);
    }

    obstacles = objects;
}

void arachnida::path_planning::artificialPotentialField::setStart(Eigen::Vector3d startPos){
    start = startPos;
}

void arachnida::path_planning::artificialPotentialField::setGoal(){
    // randomly set goal position (future work to find a better way)
    int maximumGridSize = 5;
    Eigen::Vector3d goalPos(rand() % maximumGridSize, rand() % maximumGridSize, 0);
    goal = goalPos;
}

void arachnida::path_planning::artificialPotentialField::setCurrPosition(Eigen::Vector3d currPos){
    currPosition = currPos;
}

Eigen::Vector3d arachnida::path_planning::artificialPotentialField::getGoal(void) {
    return goal;
}

// Newton direction method for generating optimal direction vector
Eigen::Vector3d arachnida::path_planning::artificialPotentialField::generateDirectionVector(void) {
    float x_ahead = currPosition.x() + stepSize;
    float y_ahead = currPosition.y() + stepSize;
    float z_ahead = currPosition.z() + stepSize;
    float x_behind = currPosition.x() - stepSize;
    float y_behind = currPosition.y() - stepSize;
    float z_behind = currPosition.z() - stepSize;

    Eigen::Vector3d x_aheadPosition(x_ahead,currPosition.y(),currPosition.z());
    Eigen::Vector3d x_behindPosition(x_behind,currPosition.y(),currPosition.z());
    Eigen::Vector3d y_aheadPosition(currPosition.x(),y_ahead,currPosition.z());
    Eigen::Vector3d y_behindPosition(currPosition.x(),y_behind,currPosition.z());
    Eigen::Vector3d xy_aheadPosition(x_ahead,y_ahead,currPosition.z());
    Eigen::Vector3d xy_behindPosition(x_behind,y_behind,currPosition.z());

    double overall_pot = calculateOverallLocalPotential(obstacles);

    double x_goal_pot_ahead = calculateGoalPotential(calculateDistanceToGoal(x_aheadPosition,goal));
    double x_goal_pot_behind = calculateGoalPotential(calculateDistanceToGoal(x_behindPosition,goal));
    double y_goal_pot_ahead = calculateGoalPotential(calculateDistanceToGoal(y_aheadPosition,goal));
    double y_goal_pot_behind = calculateGoalPotential(calculateDistanceToGoal(y_behindPosition,goal));
    double xy_goal_pot_ahead = calculateGoalPotential(calculateDistanceToGoal(xy_aheadPosition,goal));
    double xy_goal_pot_behind = calculateGoalPotential(calculateDistanceToGoal(xy_behindPosition,goal));

    double x_obs_pot_ahead = 0.0;
    double x_obs_pot_behind = 0.0;
    double y_obs_pot_ahead = 0.0;
    double y_obs_pot_behind = 0.0;
    double xy_obs_pot_ahead = 0.0;
    double xy_obs_pot_behind = 0.0;

    for (int i = 0; i < obstacles.size(); i++) {
        x_obs_pot_ahead += calculateObstaclePotential(x_aheadPosition,obstacles.at(i));
        x_obs_pot_behind += calculateObstaclePotential(x_behindPosition,obstacles.at(i));
        y_obs_pot_ahead += calculateObstaclePotential(y_aheadPosition,obstacles.at(i));
        y_obs_pot_behind += calculateObstaclePotential(y_behindPosition,obstacles.at(i));
        xy_obs_pot_ahead += calculateObstaclePotential(xy_aheadPosition,obstacles.at(i));
        xy_obs_pot_behind += calculateObstaclePotential(xy_behindPosition,obstacles.at(i));
    }

    double x_pot_ahead = x_goal_pot_ahead + x_obs_pot_ahead;
    double x_pot_behind = x_goal_pot_behind + x_obs_pot_behind;
    double y_pot_ahead = y_goal_pot_ahead + y_obs_pot_ahead;
    double y_pot_behind = y_goal_pot_behind + y_obs_pot_behind;
    double xy_pot_ahead = xy_goal_pot_ahead + xy_obs_pot_ahead;
    double xy_pot_behind = xy_goal_pot_behind + xy_obs_pot_behind;

    double v_x = (x_pot_ahead-x_pot_behind)/(2*stepSize);
    double v_y = (y_pot_ahead-y_pot_behind)/(2*stepSize);
    double v_xx = (x_pot_ahead-2*overall_pot+x_pot_behind)/pow(stepSize,2);
    double v_yy = (y_pot_ahead-2*overall_pot+y_pot_behind)/pow(stepSize,2);
    double v_xy = (xy_pot_ahead-x_pot_ahead-y_pot_ahead+2*overall_pot-x_pot_behind-y_pot_behind+xy_pot_behind)/(2*pow(stepSize,2));

    double hessian[2][2] = {
        {v_xx,v_xy},
        {v_xy,v_yy}
    };

    double gradient[2][1] = {
        {v_x},
        {v_y}
    };

    double determinant = v_xx*v_yy-pow(v_xy,2);

    if (determinant == 0) {
        Eigen::Vector3d overall_direction(0,0,0);

        return overall_direction;
    } else {
        double inverse_hessian[2][2] = {
            {(1/determinant)*v_yy,(-1/determinant)*v_xy},
            {(-1/determinant)*v_xy,(1/determinant)*v_xx}
        };

        double direction[2][1] = {
            {inverse_hessian[0][0]*gradient[0][0]+inverse_hessian[0][1]*gradient[1][0]},
            {inverse_hessian[1][0]*gradient[0][0]+inverse_hessian[1][1]*gradient[1][0]}
        };

        Eigen::Vector3d overall_direction(-1*direction[0][0],-1*direction[0][1],0);
        overall_direction = (1/sqrt(pow(direction[0][0],2)+pow(direction[0][1],2)))*overall_direction; // unit vector

        return overall_direction;
    }
}
