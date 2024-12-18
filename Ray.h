//
// Created by cai on 24-12-11.
//

#ifndef RAY_H
#define RAY_H
#include<Eigen/Dense>


class Ray {
public:
    double t;
    double tMin,tMax;
    Eigen::Vector3f origin;
    Eigen::Vector3f direction, direction_inv;

    Ray(const Eigen::Vector3f& _origin, const  Eigen::Vector3f& _direction, const double& t);

    Eigen::Vector3f operator()(const double& t) const;


};



#endif //RAY_H
