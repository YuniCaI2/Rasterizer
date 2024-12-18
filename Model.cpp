//
// Created by cai on 24-12-6.
//
#include "Model.h"
#include<unordered_map>

Model::Model() {
    rotationAngle = 0.f;
    rotationAxis << 0.f , 1.f, 0.f;
    scale << 1.f , 1.f , 1.f;
    modelFlag = 0;
}

void Model::LoadModel(std::string path) {
    loader.LoadFile(path);
    for (auto mesh : loader.LoadedMeshes) {
        std::string modelPath = "/Users/yunicai/Model/blue-archive-sunohara-kokona/";
        Texture* teX = new Texture;
        if(modelFlag == 0 ) {
            std::string mPath = modelPath + std::string(mesh.MeshMaterial.map_Kd);
            teX->Load(mPath);
            std::cout << mPath << std::endl;
        }
        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            auto t = new Triangle;
        if (modelFlag == 0) {
            t->Tex = teX;
        }
            for (int j = 0; j < 3; j++) {
                t->setVertex(j, Eigen::Vector4f(mesh.Vertices[i + j].Position.X,
                                                mesh.Vertices[i + j].Position.Y,
                                                mesh.Vertices[i + j].Position.Z, 1.0));
                t->setNormal(j, Eigen::Vector3f(mesh.Vertices[i + j].Normal.X,
                                                mesh.Vertices[i + j].Normal.Y,
                                                mesh.Vertices[i + j].Normal.Z));
                t->setTexCoord(j, Eigen::Vector2f(mesh.Vertices[i + j].TextureCoordinate.X,
                                                  mesh.Vertices[i + j].TextureCoordinate.Y));

                // std::cout << "Triangle: " << i << std::endl;
                // std::cout << t->vertex[j] << std::endl;
            }

            triangleList.push_back(t);
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
    this->scale << scale[0], scale[1], scale[2];
}

void Model::SetTexture(const Texture &texture) {
    this->texture = texture;
    modelFlag = 1 ;
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
    rotationAngle = rotationAngle * M_PI / 180.0f;
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
