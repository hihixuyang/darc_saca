#ifndef WALLVERTICES_H
#define WALLVERTICES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>

// for building dynamic number of walls
struct Vertex
{
    Eigen::Vector3f tr, tl, br, bl;
};

// pre-declaration of obstacle building function
void buildObstacles(std::vector< Vertex >& vertices, std::vector< Eigen::Vector3f >& normals, int sim, int SCENE);

#endif

