// UNFINISHED
// Supposed to define all the functionality to run various sensor
// data through Extended Kalman Filters to improve measurement
// accuracy.
#pragma once

#include <Eigen/Dense>

namespace arachnida {
    typedef struct EKF_t {
        int n; // Number of state values
        int m; // Number of observables

        Eigen::VectorXd x; // State vector -> For a pose state, this should hold its x, y, z, roll, pitch, and yaw

        Eigen::MatrixXd P; // State prediction covariance matrix
        Eigen::MatrixXd Q; // Process noise covariance matrix
        Eigen::MatrixXd R; // Measurement noise covariance matrix

        Eigen::MatrixXd G; // Kalman gain
        Eigen::MatrixXd F; // Jacobian of process model
        Eigen::MatrixXd H; // Jacobian of measurement model

        Eigen::VectorXd fx; // Output of user-defined f() state-transition function
        Eigen::VectorXd hx; // Output of user-defined h() measurement function

    } EKF_t;

    class ExtendedKalmanFilter {
        private:
            EKF_t ekf;
            int numberOfStates;
            int numberOfObservations;
            Eigen::VectorXd x; // Current state vector

        public:
            ExtendedKalmanFilter(
                int numberOfStates, 
                int numberOfObservations
            ) : numberOfStates(numberOfStates), 
                numberOfObservations(numberOfObservations) 
            {
                ekfInit(this->ekf);
                this->x = this->ekf.x;
            };

            ~ExtendedKalmanFilter() {};

            virtual void model(
                const Eigen::VectorXd fx, 
                const Eigen::MatrixXd F, 
                const Eigen::VectorXd hx, 
                const Eigen::MatrixXd H
            ) = 0;

            void ekfInit(EKF_t ekf);

            int ekfStep(
                EKF_t ekf, 
                Eigen::VectorXd &observationVec
            );

            bool step(Eigen::VectorXd &observationVec) {
                this->model(this->ekf.fx, this->ekf.F, this->ekf.hx, this->ekf.H);
                return this->ekfStep(this->ekf, observationVec) ? false : true;
            }

            EKF_t getEKF() { return ekf; };

    };
};