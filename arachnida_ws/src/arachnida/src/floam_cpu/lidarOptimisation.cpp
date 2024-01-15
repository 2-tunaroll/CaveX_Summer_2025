// This is an optimized implementation of the algorithm described in the following paper:
// 	J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
// 	Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// And this work is modified based on Advanced implementation of LOAM (A-LOAM) by 
// Tong Qin               qintonguav@gmail.com
// Shaozu Cao 		 saozu.cao@connect.ust.hk

// Author of FLOAM
// Wang Han 
// Nanyang Technological University, Singapore
// Email: wh200720041@gmail.com 
// Homepage: https://wanghan.pro

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
#include "floam_cpu/lidarOptimisation.h"

Eigen::Matrix<double,3,3> skew(Eigen::Matrix<double,3,1> &matIn) {
    Eigen::Matrix<double,3,3> skewMat;
    skewMat.setZero();
    skewMat(0,1) = -matIn(2);
    skewMat(0,2) = matIn(1);
    skewMat(1,0) = matIn(2);
    skewMat(1,2) = -matIn(0);
    skewMat(2,0) = -matIn(1);
    skewMat(2,1) = matIn(0);
    return skewMat;
}

EdgeAnalyticCostFunction::EdgeAnalyticCostFunction(Eigen::Vector3d currentPoint, Eigen::Vector3d lastPointA, Eigen::Vector3d lastPointB) : currentPoint(currentPoint), lastPointA(lastPointA), lastPointB(lastPointB) {}

bool EdgeAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> qLastCurrent(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> tLastCurrent(parameters[0] + 4);
    Eigen::Vector3d lp;
    lp = qLastCurrent * currentPoint + tLastCurrent;

    Eigen::Vector3d nu = (lp - lastPointA).cross(lp - lastPointB); // normalised unit vector???
    Eigen::Vector3d de = lastPointA - lastPointB; // de?? Idk what "de" means
    double deNormalised = de.norm();
    residuals[0] = nu.norm() / deNormalised;

    if(jacobians != NULL) {
        if(jacobians[0] != NULL) {
            Eigen::Matrix3d skewLP = skew(lp);
            Eigen::Matrix<double, 3, 6> dpBySE3;
            dpBySE3.block<3,3>(0,0) = -skewLP;
            (dpBySE3.block<3,3>(0,3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_SE3(jacobians[0]);
            J_SE3.setZero();
            Eigen::Matrix3d skewDe = skew(de);
            J_SE3.block<1,6>(0,0) = -nu.transpose() / nu.norm() * skewDe * dpBySE3/deNormalised;
        }
    }

    return true;
}

SurfNormAnalyticCostFunction::SurfNormAnalyticCostFunction(Eigen::Vector3d currentPoint, Eigen::Vector3d planeUnitNormalised, double negativeOADotNormalised) : currentPoint(currentPoint), planeUnitNormalised(planeUnitNormalised), negativeOADotNormalised(negativeOADotNormalised) {}

bool SurfNormAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> qWCurrent(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> tWCurrent(parameters[0] + 4);
    Eigen::Vector3d pointW = qWCurrent * currentPoint + tWCurrent;
    residuals[0] = planeUnitNormalised.dot(pointW) + negativeOADotNormalised;

    if(jacobians != NULL) {
        if(jacobians[0] != NULL) {
            Eigen::Matrix3d skewPointW = skew(pointW);
            Eigen::Matrix<double,3,6> dpBySE3;
            dpBySE3.block<3,3>(0,0) = -skewPointW;
            (dpBySE3.block<3,3>(0,3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double,1,7,Eigen::RowMajor>> J_SE3(jacobians[0]);
            J_SE3.setZero();
            J_SE3.block<1,6>(0,0) = planeUnitNormalised.transpose() * dpBySE3;
        }
    }

    return true;
}

void getTransformFromSE3(const Eigen::Matrix<double,6,1> &SE3, Eigen::Quaterniond &q, Eigen::Vector3d &t) {
    Eigen::Vector3d omega(SE3.data());
    Eigen::Vector3d upsilon(SE3.data()+3);
    Eigen::Matrix3d Omega = skew(omega);

    double theta = omega.norm();
    double halfTheta = 0.5*theta;

    double imagFactor; // imaginary factor
    double realFactor = cos(halfTheta); // real factor

    if(theta < 1e-10) {
        double thetaSq = theta*theta;
        double thetaPower4 = thetaSq*thetaSq;
        imagFactor = 0.5 - 0.020833*thetaSq + 0.000260417*thetaPower4;
    } else {
        double sinHalfTheta = sin(halfTheta);
        imagFactor = sinHalfTheta/theta;
    }

    q = Eigen::Quaterniond(realFactor, imagFactor*omega.x(), imagFactor*omega.y(), imagFactor*omega.z());

    Eigen::Matrix3d J;
    
    if(theta < 1e-10) {
        J = q.matrix();
    } else {
        Eigen::Matrix3d OmegaSq = Omega*Omega;
        J = (Eigen::Matrix3d::Identity() + (1-cos(theta))/(theta*theta)*Omega + (theta-sin(theta))/(theta*theta*theta)*OmegaSq);
    }

    t = J*upsilon;
}

bool PoseSE3Parameterisation::Plus(const double *x, const double *delta, double *xPlusDelta) const {
    Eigen::Map<const Eigen::Vector3d> trans(x + 4);
    Eigen::Quaterniond deltaQ;
    Eigen::Vector3d deltaT;
    getTransformFromSE3(Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), deltaQ, deltaT);
    Eigen::Map<const Eigen::Quaterniond> quater(x);
    Eigen::Map<Eigen::Quaterniond> quaterPlus(xPlusDelta);
    Eigen::Map<Eigen::Vector3d> transPlus(xPlusDelta + 4);
    quaterPlus = deltaQ * quater;
    transPlus = deltaQ * trans + deltaT;

    return true;
}

bool PoseSE3Parameterisation::ComputeJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double,7,6,Eigen::RowMajor>> J(jacobian);
    (J.topRows(6)).setIdentity();
    (J.bottomRows(1)).setZero();
    return true;
}