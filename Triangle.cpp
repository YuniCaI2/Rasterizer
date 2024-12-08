//
// Created by cai on 24-12-6.
//

#include "Triangle.h"

Triangle::Triangle() {
    vertex[0] << 0,0,0,1;
    vertex[1] << 0,0,0,1;
    vertex[2] << 0,0,0,1;

    color[0] << 0.0, 0.0, 0.0;
    color[1] << 0.0, 0.0, 0.0;
    color[2] << 0.0, 0.0, 0.0;

    tex[0] << 0.0, 0.0;
    tex[1] << 0.0, 0.0;
    tex[2] << 0.0, 0.0;
}

Eigen::Vector4f Triangle::a() {
    return vertex[0];
}

Eigen::Vector4f Triangle::b() {
    return vertex[1];
}

Eigen::Vector4f Triangle::c() {
    return vertex[2];
}

void Triangle::setVertex(int ind, const Eigen::Vector4f& ver) {
    vertex[ind] = ver;
    // std::cout << ver << std::endl;
}

void Triangle::setNormal(int ind,const Eigen::Vector3f& n) {
    normal[ind] = n;
}

void Triangle::setColor(int ind, float r, float g, float b) {
    color[ind] << r, g, b;
}

void Triangle::setNormals(const std::array<Eigen::Vector3f, 3> &normals) {
    normal[0] = normals[1];
    normal[1] = normals[2];
    normal[2] = normals[0];
}

void Triangle::setColors(const std::array<Eigen::Vector3f, 3> &colors) {
    setColor(0, colors[0][0], colors[0][1], colors[0][2]);
    setColor(1, colors[1][0], colors[1][1], colors[1][2]);
    setColor(2, colors[2][0], colors[2][1], colors[2][2]);
}

void Triangle::setTexCoord(int ind, Eigen::Vector2f uv) {
    tex[ind] = uv;
}

void Triangle::setTexCoords(const std::array<Eigen::Vector2f, 3> &tex) {
    setTexCoord(0, tex[0]);
    setTexCoord(1, tex[1]);
    setTexCoord(2, tex[2]);
}

void Triangle::setVertexs(const std::array<Eigen::Vector4f, 3> &vertexs) {
    vertex[0] = vertexs[1];
    vertex[1] = vertexs[2];
    vertex[2] = vertexs[0];
}

