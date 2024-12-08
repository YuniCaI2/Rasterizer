//
// Created by cai on 24-12-6.
//

#ifndef MODEL_H
#define MODEL_H

#include "OBJ_Loader.h"
#include "Triangle.h"


class Model {
public:
    Model();
    objl::Loader loader;
    std::vector<Triangle*> triangleList;
    Texture texture;

    Eigen::Vector3f position;
    Eigen::Vector3f rotationAxis;
    Eigen::Vector3f scale;
    float rotationAngle;

    Eigen::Matrix4f modelMatrix;

    void LoadModel(std::string path);
    void SetPosition(float x, float y, float z);
    void SetRotation(Eigen::Vector3f rotation,float angle);
    void SetScale(Eigen::Vector3f scale);
    void SetTexture(const Texture& texture);
    Eigen::Matrix4f GetModelMatrix();

    int modelFlag;


};



#endif //MODEL_H
