//
// Created by cai on 24-12-6.
//

#ifndef SHADER_H
#define SHADER_H
#include <Eigen/src/Core/Matrix.h>
#include "Fragment.h"
struct Light {
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

class Shader {

public:
    Light light{};
    Eigen::Vector3f ka;
    Eigen::Vector3f ks;
    Eigen::Vector3f eyePos;
    float Gloss;


    Shader();
    void SetLight(const Light& light);
    void SetLightPos(const Eigen::Vector3f& lightPos);
    void SetLightIntensity(const Eigen::Vector3f& Intensity);
    void SetEyePos(const Eigen::Vector3f& eyePos);
    Eigen::Vector3f GetLightPos();
    Eigen::Vector3f UsingShader(const fragment_shader_payload& fragment);
};



#endif //SHADER_H
