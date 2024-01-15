#include "object_detection_cpu/objRender.h"

int countRays = 0;

void renderRays(
    pcl::visualization::PCLVisualizer::Ptr &viewer, 
    const Vect3 &origin, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud
) {
    // render rays
    for (int i = 0; i < cloud->points.size(); i++) {
        pcl::PointXYZI point = cloud->points[i];
        pcl::PointXYZI originPoint;
        originPoint.x = origin.x;
        originPoint.y = origin.y;
        originPoint.z = origin.z;
        originPoint.intensity = 0;
        viewer->addLine<pcl::PointXYZI>(originPoint, point, 0, 0, 1, "ray" + std::to_string(countRays));
        countRays++;
    }
}

void clearRays(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    while(countRays) {
        countRays--;
        viewer->removeShape("ray" + std::to_string(countRays));
    }
}

void renderPointCloud(
    pcl::visualization::PCLVisualizer::Ptr &viewer, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, 
    std::string name, 
    Colour colour
) {
    if (colour.r == -1) {
        // Select colour based off of cloud intensity
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
        viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
    } else {
        viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colour.r, colour.g, colour.b, name);
    }
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

// void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::string name, Colour colour) {
//     if (colour.r == -1) {
//         // Select colour based off of cloud intensity
//         pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
//         viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
//     } else {
//         // Select colour based off input value
//         viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
//         viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colour.r, colour.g, colour.b, name);
//     }
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
// }

// Render wireframe box with filled transparent colour
void renderBox(
    pcl::visualization::PCLVisualizer::Ptr &viewer, 
    Box box, 
    int id, 
    Colour colour, 
    float opacity
) {
    if(opacity > 1.0) opacity = 1.0;
    else if(opacity < 0.0) opacity = 0.0;

    std::string cube = "box" + std::to_string(id);

    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, colour.r, colour.g, colour.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colour.r, colour.g, colour.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = "boxFill" + std::to_string(id);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, colour.r, colour.g, colour.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colour.r, colour.g, colour.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}

void renderBox(
    pcl::visualization::PCLVisualizer::Ptr &viewer, 
    BoxQ box, 
    int id, 
    Colour colour, 
    float opacity
) {
    if(opacity > 1.0) opacity = 1.0;
    else if(opacity < 0.0) opacity = 0.0;

    std::string cube = "box" + std::to_string(id);
    viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cubeLength, box.cubeWidth, box.cubeHeight, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colour.r, colour.g, colour.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = "boxFill" + std::to_string(id);
    viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cubeLength, box.cubeWidth, box.cubeHeight, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colour.r, colour.g, colour.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}
