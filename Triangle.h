//
// Created by cai on 24-12-6.
//
#pragma once
#ifndef TRIANGLE_H
#define TRIANGLE_H
#include <Eigen/Dense>

#include "Texture.h"

class Triangle {

public:

    Triangle();
    Texture *Tex= nullptr;

    Eigen::Vector4f vertex[3];
    Eigen::Vector3f normal[3];
    Eigen::Vector2f tex[3];
    Eigen::Vector3f color[3];



public:
    Eigen::Vector4f a();
    Eigen::Vector4f b();
    Eigen::Vector4f c();

    void setVertex(int ind,const Eigen::Vector4f& ver);
    void setNormal(int ind,const Eigen::Vector3f& n);
    void setColor(int ind, float r, float g, float b);

    void setNormals(const std::array<Eigen::Vector3f, 3>& normals);
    void setColors(const std::array<Eigen::Vector3f, 3>& colors);
    void setTexCoord(int ind, Eigen::Vector2f uv );
    void setTexCoords(const std::array<Eigen::Vector2f, 3>& tex);
    void setVertexs(const std::array<Eigen::Vector4f, 3>& vertexs);

    // Eigen::Vector4f toVector4();

};



#endif //TRIANGLE_H
