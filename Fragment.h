//
// Created by cai on 24-12-6.
//

#ifndef FRAGMENT_H
#define FRAGMENT_H
#include "Texture.h"

struct fragment_shader_payload
{
    fragment_shader_payload()
    {
        texture = nullptr;
    }

    fragment_shader_payload(const Eigen::Vector3f& col, const Eigen::Vector3f& nor,const Eigen::Vector2f& tc, Texture* tex) :
         color(col), normal(nor), texCoord(tc), texture(tex) {}


    Eigen::Vector3f view_pos;
    Eigen::Vector3f color;
    Eigen::Vector3f normal;
    Eigen::Vector2f texCoord;
    Texture* texture;
};
#endif //FRAGMENT_H
