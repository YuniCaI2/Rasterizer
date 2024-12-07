//
// Created by cai on 24-12-6.
//

#include "Shader.h"

Shader::Shader() {
    ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);
    light.position = Eigen::Vector3f(20.0f, 20.0f, 20.0f);
    light.intensity = Eigen::Vector3f(500.0, 500.0, 500.0);
    eyePos = Eigen::Vector3f(0.0, 0.0, 0.0);
    Gloss =64;
}

void Shader::SetLight(const Light &light) {
    this->light.position = light.position;
    this->light.intensity = light.intensity;
}

void Shader::SetLightPos(const Eigen::Vector3f &lightPos) {
    light.position = lightPos;
}

void Shader::SetLightIntensity(const Eigen::Vector3f &Intensity) {
    light.intensity = Intensity;
}

void Shader::SetEyePos(const Eigen::Vector3f &eyePos) {
    this->eyePos = eyePos;
}

Eigen::Vector3f Shader::GetLightPos() {
    return Eigen::Vector3f(light.position.x(), light.position.y(), light.position.z());
}

Eigen::Vector3f Shader::UsingShader(const fragment_shader_payload &fragment) {
    Eigen::Vector3f return_color = {0.5, 0.5, 0.5};
    if (fragment.texture)
    {
        float u = std::clamp(static_cast<double>(fragment.texCoord(0)),0.0,1.0);
        float v = std::clamp(static_cast<double>(fragment.texCoord(1)),0.0,1.0);
        return_color = fragment.texture->getColor(u,v);
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();
    Eigen::Vector3f kd = texture_color;

    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos = eyePos;

    float p = Gloss;

    Eigen::Vector3f point = fragment.view_pos;
    Eigen::Vector3f normal = fragment.normal;

    Eigen::Vector3f result_color = {0, 0, 0};



        // components are. Then, accumulate that result on the *result_color* object.
        // Light Direction
    Eigen::Vector3f light_dir = (light.position - point).normalized();
        // View Direction
    Eigen::Vector3f view_dir = (eye_pos - point).normalized();
        // Half Vector
    Eigen::Vector3f h = (light_dir + view_dir).normalized();
        // Light Attenuation
    Eigen::Vector3f attenuated_light = light.intensity / ((light.position - point).norm() * (light.position - point).norm() + 1e-4);
        // Ambient
    Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);
        // Diffuse
    Eigen::Vector3f diffuse = kd.cwiseProduct(attenuated_light) * std::max(0.0f,normal.dot(light_dir));
        // Specular
    Eigen::Vector3f specular = ks.cwiseProduct(attenuated_light) * std::pow(std::max(0.0f,normal.dot(h)),p);

    result_color += (diffuse + ambient + specular);


    return result_color * 255.f;
}



