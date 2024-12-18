//
// Created by cai on 24-12-6.
//

#include "Texture.h"

Texture::Texture(const std::string &path) {
    image = cv::imread(path);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    w = image.cols;
    h = image.rows;
}


void Texture::Load(const std::string &path) {
    image = cv::imread(path);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    w = image.cols;
    h = image.rows;
}

Eigen::Vector3f Texture::getColor(float u, float v) {
    auto u_img = u * w;
    auto v_img = (1 - v) * h;
    auto color = image.at<cv::Vec3b>(v_img, u_img);
    return Eigen::Vector3f(color[0], color[1], color[2]);
}

Eigen::Vector3f Texture::getColorBilinear(float u, float v) {

    auto u_img = u * w;
    auto v_img = (1 - v) * h;

    cv::Mat patch;
    cv::getRectSubPix(image,cv::Size(1,1),cv::Point2f(u_img,v_img),patch);
    auto color = patch.at<cv::Vec3b>(0,0);

    return Eigen::Vector3f(color[0],color[1],color[2]);
}
