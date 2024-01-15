#include "object_detection_cpu/objRansac.h"
#include <iostream>
#include <chrono>

std::unordered_set<int> ransacPlane(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
    const int maxIterations, 
    const float distanceTolerance
) {
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    auto t1 = std::chrono::high_resolution_clock::now();

    const int pointsToFit = 3;
    for(int i = 0; i < maxIterations; i++) {
        // Select three random points
        std::unordered_set<int> inliers;
        for(int j = 0; j < pointsToFit; j++) {
            inliers.insert((rand() % cloud->points.size()));
        }
        if(inliers.size() > 2) {

            float x1, y1, z1;
            float x2, y2, z2;
            float x3, y3, z3;

            auto itr = inliers.begin();

            x1 = cloud->points[*itr].x;
            y1 = cloud->points[*itr].y;
            z1 = cloud->points[*itr].z;

            itr++;
            x2 = cloud->points[*itr].x;
            y2 = cloud->points[*itr].y;
            z2 = cloud->points[*itr].z;

            itr++;

            x3 = cloud->points[*itr].x;

            y3 = cloud->points[*itr].y;

            z3 = cloud->points[*itr].z;

            // Calculate the plane coefficients (a, b, c, d)
            // Equation of a plane: Ax + By + Cz - d = 0

            const float a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
            const float b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
            const float c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
            const float d = -(a*x1 + b*y1 + c*z1);


            for(int k = 0; k < cloud->points.size(); k++) {
                if(inliers.count(k) > 0) continue;

                pcl::PointXYZI p = cloud->points[k];
                float x4 = p.x;
                float y4 = p.y;
                float z4 = p.z;

                // Estimate the distance of each point and check if it's within the distanceTolerance
                const float D = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c*c);
                if(D < distanceTolerance)  {
                    inliers.insert(k);
                }
            }
        }

        // The fit with the most inliers will be selected
        if(inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    std::chrono::duration<double, std::milli> ms_double = t2 - t1;
    std::cout << "ransac duration: " << dur.count() << "ms\n";

    return inliersResult;
}