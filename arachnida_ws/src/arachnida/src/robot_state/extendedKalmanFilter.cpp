// UNFINISHED
// Supposed to define all the functionality to run various sensor
// data through Extended Kalman Filters to improve measurement
// accuracy.
#include "robot_state/extendedKalmanFilter.h"

void arachnida::ExtendedKalmanFilter::ekfInit(EKF_t ekf) {   
    // Initialise matrices to zeroes
    this->ekf.P = Eigen::MatrixXd::Zero(this->ekf.n, this->ekf.n);
    this->ekf.Q = Eigen::MatrixXd::Zero(this->ekf.n, this->ekf.n);
    this->ekf.R = Eigen::MatrixXd::Zero(this->ekf.m, this->ekf.m);
    this->ekf.G = Eigen::MatrixXd::Zero(this->ekf.n, this->ekf.m);
    this->ekf.F = Eigen::MatrixXd::Zero(this->ekf.n, this->ekf.n);
    this->ekf.H = Eigen::MatrixXd::Zero(this->ekf.m, this->ekf.n);
};

int arachnida::ExtendedKalmanFilter::ekfStep(
    EKF_t ekf, 
    Eigen::VectorXd &observationVec
) {
    // Predict
    ekf.x = ekf.fx;
    ekf.P = ekf.F * ekf.P * ekf.F.transpose() + ekf.Q;

    // Update
    ekf.G = ekf.P * ekf.H.transpose() * (ekf.H * ekf.P * ekf.H.transpose() + ekf.R).inverse();
    ekf.x = ekf.x + ekf.G * (observationVec - ekf.hx);
    ekf.P = ekf.P - ekf.G * ekf.H * ekf.P;

    return 0;
};

void arachnida::ExtendedKalmanFilter::model(
    const Eigen::VectorXd fx, 
    const Eigen::MatrixXd F, 
    const Eigen::VectorXd hx, 
    const Eigen::MatrixXd H
) {

};