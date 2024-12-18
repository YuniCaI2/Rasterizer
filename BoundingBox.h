//
// Created by cai on 24-12-11.
//

#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H
#include <Eigen/Dense>
#include "Ray.h"

class BoundingBox {
public:
    Eigen::Vector3f pMax, pMin;
    //两个角点关于包围盒

    BoundingBox();
    BoundingBox(Eigen::Vector3f pMin, Eigen::Vector3f pMax);

    Eigen::Vector3f Diagonal() const;

    Eigen::Vector3f GetCenter() const;

    BoundingBox Merge(const BoundingBox &other) const;

    bool Inside(const Eigen::Vector3f &p) const;
    bool IntersectP(const Ray& ray, const Eigen::Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const;
};



#endif //BOUNDINGBOX_H
