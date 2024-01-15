#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vector>
#include <string>
#include "objBox.h"

struct Colour {
    float r, g, b;

    Colour(
        float R, 
        float G, 
        float B
    ) : r(R), 
        g(G), 
        b(B) {}
};

struct Vect3 {
    double x, y, z;
    
    Vect3(
        double X, 
        double Y, 
        double Z
    ) : x(X), 
        y(Y), 
        z(Z) {}

    Vect3 operator+(const Vect3 &v) {
        return Vect3(x + v.x, y + v.y, z + v.z);
    }
};

enum CameraAngle {
    XY, TopDown, Side, FPS
};

struct Object { // originally the "Car" struct
    Vect3 position, dimensions;
    std::string name;
    Colour colour;

    Object(
        Vect3 position, 
        Vect3 dimensions, 
        Colour colour, 
        std::string name
    ) : position(position), 
        dimensions(dimensions), 
        colour(colour), 
        name(name) {}

    void render(pcl::visualization::PCLVisualizer::Ptr &viewer) {

        // render bottom of object
        viewer->addCube(position.x - dimensions.x / 2, position.x + dimensions.x / 2, position.y - dimensions.y / 2, position.y + dimensions.y / 2, position.z - dimensions.z / 2, position.z + dimensions.z / 2, colour.r, colour.g, colour.b, name);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colour.r, colour.g, colour.b, name);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name);

        // render top of object
        viewer->addCube(position.x - dimensions.x / 4, position.x + dimensions.x / 4, position.y - dimensions.y / 2, position.y + dimensions.y / 2, position.z - dimensions.z / 2, position.z + dimensions.z / 2, colour.r, colour.g, colour.b, name + "Top");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name + "Top");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colour.r, colour.g, colour.b, name + "Top");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name + "Top");
    }

    bool inbetween(
        double point, 
        double centre, 
        double range
    ) {
        return (centre - range <= point) && (centre + range >= point);
    }

    bool checkCollision(Vect3 point) {
        return (inbetween(point.x, position.x, dimensions.x / 2) && inbetween(point.y, position.y, dimensions.y / 2) && inbetween(point.z, position.z + dimensions.z / 3, dimensions.z / 3)) 
            || (inbetween(point.x, position.x, dimensions.x / 4) && inbetween(point.y, position.y, dimensions.y / 2) && inbetween(point.z, position.z + dimensions.z * 5 / 6, dimensions.z / 6));
    }
};

void renderRays(
    pcl::visualization::PCLVisualizer::Ptr &viewer, 
    const Vect3 &origin, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud
);

void clearRays(pcl::visualization::PCLVisualizer::Ptr &viewer);

void renderPointCloud(
    pcl::visualization::PCLVisualizer::Ptr &viewer, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, 
    std::string name, 
    Colour colour = Colour(1, 1, 1)
);
// void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::string name, Colour colour = Colour(-1, -1, -1));

void renderBox(
    pcl::visualization::PCLVisualizer::Ptr &viewer, 
    Box box, 
    int id, 
    Colour colour = Colour(1, 0, 0), 
    float opacity = 1
);

void renderBox(
    pcl::visualization::PCLVisualizer::Ptr &viewer, 
    BoxQ box, 
    int id, 
    Colour colour = Colour(1, 0, 0), 
    float opacity = 1
);