//
// Created by cai on 24-12-6.
//

#ifndef RASTERIZER_H
#define RASTERIZER_H
#include<Eigen/Dense>
#include <utility>

#include "Model.h"
#include "Shader.h"


struct Vertex {
    Eigen::Vector4f position;
    Eigen::Vector3f normal;
    Eigen::Vector2f texcoord;
    Eigen::Vector3f color;
    Eigen::Vector3f worldPos;
    Vertex(Eigen::Vector4f p, Eigen::Vector3f n, Eigen::Vector2f tex,Eigen::Vector3f c,Eigen::Vector3f wordP) {
        position = std::move(p);
        normal = std::move(n);
        texcoord = std::move(tex);
        color = std::move(c);
        worldPos = std::move(wordP);
    }
};

class rasterizer {
public:

    rasterizer(int w,int h);

    Eigen::Matrix4f view;
    Eigen::Matrix4f projection;

    std::vector<Model> models;

    Shader fragmentShader;


    std::vector<Eigen::Vector3f> frameBuf;
    std::vector<float> depthBuf;
    int w, h;


    void SetView(const Eigen::Matrix4f& v);
    void SetProjection(const Eigen::Matrix4f& p);

    void SetModels(const std::vector<Model>& models);
    void SetFragmentShader(const Shader& shader);
    void ClearDepthBuffer();
    void ClearColorBuffer();

    void Draw();

    void RasterizeTriangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& worldPos);


};



#endif //RASTERIZER_H
