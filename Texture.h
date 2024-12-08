//
// Created by cai on 24-12-6.
//

#ifndef TEXTURE_H
#define TEXTURE_H
#include<Eigen/Dense>
#include<opencv2/opencv.hpp>


class Texture {
public:
    cv::Mat image;
    int w,h;

    Texture(const std::string& path);
    Texture(){};

    Eigen::Vector3f getColor(float u, float v);

    Eigen::Vector3f getColorBilinear(float u, float v);

};



#endif //TEXTURE_H
