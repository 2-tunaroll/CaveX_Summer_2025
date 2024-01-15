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
#include "floam_cpu/odomEstimationClass.h"

// Good explanation of the logic behind this class and lidarOptimisation.cpp:
// https://openaccess.thecvf.com/content_ICCV_2019/papers/Bhattacharya_Efficient_and_Robust_Registration_on_the_3D_Special_Euclidean_Group_ICCV_2019_paper.pdf

void odomEstimationClass::init(double mapResolution) {
    laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    downSizeFilterEdge.setLeafSize(mapResolution, mapResolution, mapResolution);
    downSizeFilterSurf.setLeafSize(mapResolution*2, mapResolution*2, mapResolution*2);

    kdTreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdTreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

    odom = Eigen::Isometry3d::Identity();
    lastOdom = Eigen::Isometry3d::Identity();
    optimisationCount = 2;
}

void odomEstimationClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edgeIn, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surfIn) {
    *laserCloudCornerMap += *edgeIn;
    *laserCloudSurfMap += *surfIn;
    optimisationCount = 12;
}

void odomEstimationClass::updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edgeIn, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surfIn) {
    if(optimisationCount > 2) optimisationCount--;

    Eigen::Isometry3d odomPrediction = odom*(lastOdom.inverse()*odom);
    lastOdom = odom;
    odom = odomPrediction;

    qWCurrent = Eigen::Quaterniond(odom.rotation());
    tWCurrent = odom.translation();

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZI>());
    downSamplingToMap(edgeIn, downsampledEdgeCloud, surfIn, downsampledSurfCloud);

    if(laserCloudCornerMap->points.size() > 10 && laserCloudSurfMap->points.size() > 50) {
        kdTreeEdgeMap->setInputCloud(laserCloudCornerMap);
        kdTreeSurfMap->setInputCloud(laserCloudSurfMap);

        for(int i = 0; i < optimisationCount; i++) {
            ceres::LossFunction *lossFunction = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problemOptions;
            ceres::Problem problem(problemOptions);

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterisation());

            addEdgeCostFactor(downsampledEdgeCloud, laserCloudCornerMap, problem, lossFunction);
            addSurfCostFactor(downsampledSurfCloud, laserCloudSurfMap, problem, lossFunction);

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            // options.max_num_iterations = 5;
            // options.minimizer_progress_to_stdout = true;
            options.check_gradients = false;
            // options.check_gradients = true;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;
            // std::cout << "SOLVING ----------------------- START\n";
            ceres::Solve(options, &problem, &summary);
            // std::cout << "SOLVING ----------------------- END\n";
        }
    } else std::cout << "Not enough points in map to optimise\n";

    odom = Eigen::Isometry3d::Identity();
    odom.linear() = qWCurrent.toRotationMatrix();
    odom.translation() = tWCurrent;
    addPointsToMap(downsampledEdgeCloud, downsampledSurfCloud);
}

void odomEstimationClass::pointAssociateToMap(pcl::PointXYZI const *const pIn, pcl::PointXYZI *const pOut) {
    Eigen::Vector3d pointCurr(pIn->x, pIn->y, pIn->z);
    Eigen::Vector3d pointW = qWCurrent*pointCurr + tWCurrent;
    pOut->x = pointW.x();
    pOut->y = pointW.y();
    pOut->z = pointW.z();
    pOut->intensity = pIn->intensity;
}

void odomEstimationClass::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edgePCIn, pcl::PointCloud<pcl::PointXYZI>::Ptr &edgePCOut, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surfPCIn, pcl::PointCloud<pcl::PointXYZI>::Ptr &surfPCOut) {
    downSizeFilterEdge.setInputCloud(edgePCIn);
    downSizeFilterEdge.filter(*edgePCOut);
    downSizeFilterSurf.setInputCloud(surfPCIn);
    downSizeFilterSurf.filter(*surfPCOut);
}

void odomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, const pcl::PointCloud<pcl::PointXYZI>::Ptr &mapIn, ceres::Problem &problem, ceres::LossFunction *lossFunction) {
    int cornerNum = 0;
    for(int i = 0; i < (int) pcIn->points.size(); i++) {
        // std::cout << "1\n";
        pcl::PointXYZI pointTemp;
        pointAssociateToMap(&(pcIn->points[i]), &pointTemp);

        // std::cout << "2\n";
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis; // squared distance I think

        kdTreeEdgeMap->nearestKSearch(pointTemp, 5, pointSearchInd, pointSearchSqDis);
        // std::cout << "3\n";
        if(pointSearchSqDis[4] < 1.0) {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0,0,0);
            for(int j = 0; j < 5; j++) {
                Eigen::Vector3d tmp(mapIn->points[pointSearchInd[j]].x, mapIn->points[pointSearchInd[j]].y, mapIn->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }

            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero(); // covariant matrix?
            for(int j = 0; j < 5; j++) {
                Eigen::Matrix<double,3,1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }


            // std::cout << "4\n";


            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat); // saes = self adjoint eigen solver (computs eigenvalues and eigenvectors of self adjoint matrix)
            Eigen::Vector3d unitDirection = saes.eigenvectors().col(2);
            Eigen::Vector3d currentPoint(pcIn->points[i].x, pcIn->points[i].y, pcIn->points[i].z);
            if(saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) { // if one eigen vector is much larger than the other then it indicates that the points of an edge feature are distributed in a line. Then, this means we can use the associated eigenvector as the direction of the line 
                Eigen::Vector3d pointOnLine = center;
                Eigen::Vector3d pointA, pointB;
                pointA = 0.1 * unitDirection + pointOnLine;
                pointB = -0.1 * unitDirection + pointOnLine;

                // std::cout << "5\n";
                ceres::CostFunction *costFunction = new EdgeAnalyticCostFunction(currentPoint, pointA, pointB);
                problem.AddResidualBlock(costFunction, lossFunction, parameters);
                cornerNum++;
            }
        }
    }
    // if(cornerNum < 20) std::cout << "Not enough correct points (1)\n";
    // else std::cout << "ENOUGH CORRECT POINTS (1)\n";
}

void odomEstimationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcIn, const pcl::PointCloud<pcl::PointXYZI>::Ptr &mapIn, ceres::Problem &problem, ceres::LossFunction *lossFunction) {
    int surfNum = 0;
    for(int i = 0; i < (int) pcIn->points.size(); i++) {
        pcl::PointXYZI pointTemp;
        pointAssociateToMap(&(pcIn->points[i]), &pointTemp);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdTreeSurfMap->nearestKSearch(pointTemp, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if(pointSearchSqDis[4] < 1.0) {
            for(int j = 0; j < 5; j++) {
                matA0(j, 0) = mapIn->points[pointSearchInd[j]].x;
                matA0(j, 0) = mapIn->points[pointSearchInd[j]].y;
                matA0(j, 0) = mapIn->points[pointSearchInd[j]].z;
            }

            // Find the norm of the plane
            Eigen::Vector3d planeNorm = matA0.colPivHouseholderQr().solve(matB0);
            double negativeOADotNorm = 1 / planeNorm.norm();
            planeNorm.normalize();

            bool planeValid = true;
            for(int j = 0; j < 5; j++) {
                // if OX * n > 0.2, then the plane does not fit well
                if(fabs(planeNorm(0) * mapIn->points[pointSearchInd[j]].x + planeNorm(1) * mapIn->points[pointSearchInd[j]].y + planeNorm(2) * mapIn->points[pointSearchInd[j]].z + negativeOADotNorm) > 0.2) {
                    planeValid = false;
                    break;
                }
            }
            
            Eigen::Vector3d currentPoint(pcIn->points[i].x, pcIn->points[i].y, pcIn->points[i].z);
            if(planeValid) {
                ceres::CostFunction *costFunction = new SurfNormAnalyticCostFunction(currentPoint, planeNorm, negativeOADotNorm);
                problem.AddResidualBlock(costFunction, lossFunction, parameters);
                surfNum++;
            }
        }
    }
    if(surfNum < 20) std::cout << "Not enough correct points (2)\n";
}

void odomEstimationClass::addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledSurfCloud) {
    for(int i = 0; i < (int) downsampledEdgeCloud->points.size(); i++) {
        pcl::PointXYZI pointTemp;
        pointAssociateToMap(&downsampledEdgeCloud->points[i], &pointTemp);
        laserCloudCornerMap->push_back(pointTemp);
    }

    for(int i = 0; i < (int) downsampledSurfCloud->points.size(); i++) {
        pcl::PointXYZI pointTemp;
        pointAssociateToMap(&downsampledSurfCloud->points[i], &pointTemp);
        laserCloudSurfMap->push_back(pointTemp);
    }

    double xMin = +odom.translation().x() - 100;
    double yMin = +odom.translation().y() - 100;
    double zMin = +odom.translation().z() - 100;
    double xMax = +odom.translation().x() + 100;
    double yMax = +odom.translation().y() + 100;
    double zMax = +odom.translation().z() + 100;

    cropBoxFilter.setMin(Eigen::Vector4f(xMin, yMin, zMin, 1.0));
    cropBoxFilter.setMax(Eigen::Vector4f(xMax, yMax, zMax, 1.0));
    cropBoxFilter.setNegative(false);

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCorner(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZI>());
    cropBoxFilter.setInputCloud(laserCloudSurfMap);
    cropBoxFilter.filter(*tmpSurf);
    cropBoxFilter.setInputCloud(laserCloudCornerMap);
    cropBoxFilter.filter(*tmpCorner);

    downSizeFilterSurf.setInputCloud(tmpSurf);
    downSizeFilterSurf.filter(*laserCloudSurfMap);
    downSizeFilterEdge.setInputCloud(tmpCorner);
    downSizeFilterEdge.filter(*laserCloudCornerMap);
}

void odomEstimationClass::getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr &laserCloudMap) {
    *laserCloudMap += *laserCloudSurfMap;
    *laserCloudMap += *laserCloudCornerMap;
}

odomEstimationClass::odomEstimationClass() {}