//
// Created by cai on 24-12-11.
//

#ifndef BVH_H
#define BVH_H
#include "BoundingBox.h"
#include "Model.h"


class BVH {
public:
    BVH(const std::vector<Model*> & models, const int& maxModelInNode);

    struct BVHBuildNode {
        BoundingBox bounds;
        BVHBuildNode *left;
        BVHBuildNode *right;
        Model* model;

    public:
        int splitAxis=0, firstPrimOffset=0, nPrimitives=0;
        BVHBuildNode(){
            bounds = BoundingBox();
            left = nullptr;right = nullptr;
            model = nullptr;
        }
    };

    int maxModelInNode;
    std::vector<Model*> models;

};



#endif //BVH_H
