//
// Created by cai on 24-12-6.
//
#include "Model.h"

void Model::LoadModel(std::string path) {
    loader.LoadFile(path);
    for (auto mesh : loader.LoadedMeshes) {
        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            // 使用 std::make_unique 创建 Triangle 对象
            auto t = std::make_unique<Triangle>();
            for (int j = 0; j < 3; j++) {
                t->setVertex(j, Eigen::Vector4f(mesh.Vertices[i + j].Position.X,
                                                mesh.Vertices[i + j].Position.Y,
                                                mesh.Vertices[i + j].Position.Z, 1.0));
                t->setNormal(j, Eigen::Vector3f(mesh.Vertices[i + j].Normal.X,
                                                mesh.Vertices[i + j].Normal.Y,
                                                mesh.Vertices[i + j].Normal.Z));
                t->setTexCoord(j, Eigen::Vector2f(mesh.Vertices[i + j].TextureCoordinate.X,
                                                  mesh.Vertices[i + j].TextureCoordinate.Y));
            }
            // 使用 std::move 将 t 转移到 triangleList
            triangleList.push_back(std::move(t));
        }
    }
}

void Model::SetPosition(float x, float y, float z) {
    position << x, y, z;
}

void Model::SetRotation(Eigen::Vector3f rotation, float angle) {
    rotationAxis << rotation[0], rotation[1], rotation[2];
    rotationAngle = angle;
}


void Model::SetScale(Eigen::Vector3f scale) {
    scale << scale[0], scale[1], scale[2];
}

void Model::SetTexture(const Texture &texture) {
    this->texture = texture;
}

Eigen::Matrix4f Model::GetModelMatrix() {
    Eigen::Matrix4f translationMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f scalingMatrix = Eigen::Matrix4f::Identity();

    // 平移矩阵
    translationMatrix(0, 3) = position.x();
    translationMatrix(1, 3) = position.y();
    translationMatrix(2, 3) = position.z();

    // 计算旋转矩阵
    Eigen::AngleAxisf angleAxis(rotationAngle, rotationAxis.normalized());
    rotationMatrix.block<3, 3>(0, 0) = angleAxis.toRotationMatrix();

    // 缩放矩阵
    scalingMatrix(0, 0) = scale.x();
    scalingMatrix(1, 1) = scale.y();
    scalingMatrix(2, 2) = scale.z();

    // 组合变换矩阵：T * R * S
    modelMatrix = translationMatrix * rotationMatrix * scalingMatrix;

    return modelMatrix;
}
