//
// Created by cai on 24-12-11.
//

#include "Ray.h"


Ray::Ray(const Eigen::Vector3f &_origin, const Eigen::Vector3f &_direction, const double &t) {
    origin = _origin;
    direction = _direction;
    this->t = t;
    direction_inv = Eigen::Vector3f(1 / direction.x(), 1 / direction.y(), 1 / direction.z());
    tMin = 0.0;
    tMax = std::numeric_limits<double>::max();
}

Eigen::Vector3f Ray::operator()(const double &t) const {
    return origin + direction * t;
}
