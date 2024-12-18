//
// Created by cai on 24-12-11.
//

#include "BoundingBox.h"

BoundingBox::BoundingBox() {
    float min = std::numeric_limits<float>::min();
    float max = std::numeric_limits<float>::max();

    pMax << max , max , max;
    pMin << min , min , min;

}

BoundingBox::BoundingBox(Eigen::Vector3f pMin, Eigen::Vector3f pMax) {
    this->pMin = pMin;
    this->pMax = pMax;
}

Eigen::Vector3f BoundingBox::Diagonal() const {
    return pMax - pMin;
}

Eigen::Vector3f BoundingBox::GetCenter() const {
    return (pMin + pMax) * 0.5f;
}

BoundingBox BoundingBox::Merge(const BoundingBox &other) const {
    Eigen::Vector3f newMin = pMin.cwiseMin(other.pMin);
    Eigen::Vector3f newMax = pMax.cwiseMax(other.pMax);
    return BoundingBox(newMin, newMax);
}

bool BoundingBox::Inside(const Eigen::Vector3f &p) const {
    return (p.x() >= pMin.x() && p.x() <= pMax.x() &&
               p.y() >= pMin.y() && p.y() <= pMax.y() &&
               p.z() >= pMin.z() && p.z() <= pMax.z());
}

 bool BoundingBox::IntersectP(const Ray &ray, const Eigen::Vector3f &invDir, const std::array<int, 3> &dirIsNeg) const {

    float tmin_x = (dirIsNeg[0] ? (pMax.x() - ray.origin.x()) : (pMin.x() - ray.origin.x())) * invDir.x();
    float tmax_x = (dirIsNeg[0] ? (pMin.x() - ray.origin.x()) : (pMax.x() - ray.origin.x())) * invDir.x();

    float tmin_y = (dirIsNeg[1] ? (pMax.y() - ray.origin.y()) : (pMin.y() - ray.origin.y())) * invDir.y();
    float tmax_y = (dirIsNeg[1] ? (pMin.y() - ray.origin.y()) : (pMax.y() - ray.origin.y())) * invDir.y();

    float tmin = std::max(tmin_x, tmin_y);
    float tmax = std::min(tmax_x, tmax_y);

    float tmin_z = (dirIsNeg[2] ? (pMax.z() - ray.origin.z()) : (pMin.z() - ray.origin.z())) * invDir.z();
    float tmax_z = (dirIsNeg[2] ? (pMin.z() - ray.origin.z()) : (pMax.z() - ray.origin.z())) * invDir.z();

    tmin = std::max(tmin, tmin_z);
    tmax = std::min(tmax, tmax_z);

    return tmin <= tmax && tmax >= ray.tMin && tmin <= ray.tMax;
}



