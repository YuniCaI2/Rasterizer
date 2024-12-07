//
// Created by cai on 24-12-6.
//

#include "rasterizer.h"

const std::vector<Eigen::Vector4f> ViewLines = {
    Eigen::Vector4f(0,0,-1,1),
    Eigen::Vector4f(0,0,1,1),
    Eigen::Vector4f(1,0,0,1),
    Eigen::Vector4f(-1,0,0,1),
    Eigen::Vector4f(0,1,0,1),
    Eigen::Vector4f(0,-1,0,1)

};

bool Inside(const Eigen::Vector4f &line,const Eigen::Vector4f &p) {
    return line.x() * p.x() + line.y() * p.y() + line.z() * p.z() + line.w() * p.w() >= 0;
}

float IntersectWeight(const Eigen::Vector4f &line,const Eigen::Vector4f& v1, const Eigen::Vector4f& v2) {
    float d1 = fabs(line.x() * v1.x() + line.y() * v1.y() + line.z() * v1.z() + line.w() * v1.w());
    float d2 = fabs(line.x() * v2.x() + line.y() * v2.y() + line.z() * v2.z() + line.w() * v2.w());

    return d1 / (d1 + d2);
}

bool AllInside(const Triangle& triangle) {
    Eigen::Vector4f left = Eigen::Vector4f(1,0,0,1);
    Eigen::Vector4f right = Eigen::Vector4f(-1,0,0,1);
    Eigen::Vector4f bottom = Eigen::Vector4f(0,1,0,1);
    Eigen::Vector4f top = Eigen::Vector4f(0,-1,0,1);
    Eigen::Vector4f far = Eigen::Vector4f(0,0,1,1);
    Eigen::Vector4f near = Eigen::Vector4f(0,0,-1,1);

    bool check[3];
    int i = 0;
    for(const auto& vertex : triangle.vertex ) {
        check[i] = Inside(vertex, left) && Inside(vertex, right) && Inside(vertex, bottom) && Inside(vertex, top) && Inside(vertex, far) && Inside(vertex, near);
        i++;
    }
    return check[0] && check[1] && check[2];
}


std::vector<Triangle> SutherlandHodgeman(const Triangle& triangle, std::vector<Eigen::Vector4f>& worldPoses) {
    std::vector<Triangle> resultTriangles;
    std::vector<Eigen::Vector4f> newWordPos;
    if(AllInside(triangle)) {
        resultTriangles.push_back(triangle);
        return resultTriangles;
    }
    std::vector<Vertex> resultVertices;
    resultVertices.emplace_back(triangle.vertex[0], triangle.normal[0],triangle.tex[0],triangle.color[0],worldPoses[0]);
    resultVertices.emplace_back(triangle.vertex[1], triangle.normal[1],triangle.tex[1],triangle.color[1],worldPoses[1]);
    resultVertices.emplace_back(triangle.vertex[2], triangle.normal[2],triangle.tex[2],triangle.color[2],worldPoses[2]);
    worldPoses.clear();
    for(int i = 0; i < ViewLines.size(); i++) {
        std::vector<Vertex> vertices(resultVertices);
        resultVertices.clear();
        for(int j = 0; j < vertices.size(); j++) {
            Vertex current = vertices[j];
            Vertex last = vertices[(j + vertices.size() - 1) % vertices.size()];
            if(Inside(ViewLines[i],current.position)) {
                if(! Inside(ViewLines[i],last.position)) {
                    float w = IntersectWeight(ViewLines[i],last.position,current.position);
                    Eigen::Vector4f tp = current.position * w + last.position * (1 - w);
                    Eigen::Vector3f tn = current.normal * w + last.normal * (1 - w);
                    Eigen::Vector2f tt = current.texcoord * w + last.texcoord * (1 - w);
                    Eigen::Vector3f tc = current.color * w + last.color * (1 - w);
                    Eigen::Vector3f tw = current.worldPos * w + last.worldPos * (1 - w);
                    resultVertices.emplace_back(tp,tn,tt,tc,tw);
                }
                resultVertices.push_back(current);
            }
            else if(Inside(ViewLines[i],last.position)) {
                float w = IntersectWeight(ViewLines[i],last.position,current.position);
                Eigen::Vector4f tp = current.position * w + last.position * (1 - w);
                Eigen::Vector3f tn = current.normal * w + last.normal * (1 - w);
                Eigen::Vector2f tt = current.texcoord * w + last.texcoord * (1 - w);
                Eigen::Vector3f tc = current.color * w + last.color * (1 - w);
                Eigen::Vector3f tw = current.worldPos * w + last.worldPos * (1 - w);
                resultVertices.emplace_back(tp,tn,tt,tc,tw);
            }

        }
    }
    for(int i = 0; i < resultVertices.size()-2; i++) {
        Triangle tempTriangle;
        tempTriangle.setVertexs(std::array<Eigen::Vector4f, 3>{resultVertices[0].position,resultVertices[i+1].position,resultVertices[i+2].position});
        tempTriangle.setNormals(std::array<Eigen::Vector3f, 3>{resultVertices[0].normal,resultVertices[i+1].normal,resultVertices[i+2].normal});
        tempTriangle.setTexCoords(std::array<Eigen::Vector2f, 3>{resultVertices[0].texcoord,resultVertices[i+1].texcoord,resultVertices[i+2].texcoord});
        tempTriangle.setColors(std::array<Eigen::Vector3f, 3>{resultVertices[0].color,resultVertices[i+1].color,resultVertices[i+2].color});
        worldPoses.emplace_back(resultVertices[0].worldPos);
        worldPoses.emplace_back(resultVertices[i+1].worldPos);
        worldPoses.emplace_back(resultVertices[i+2].worldPos);
        resultTriangles.push_back(tempTriangle);
    }


    return resultTriangles;

}



static bool InsideTriangle(int x, int y, const Eigen::Vector4f* _v) {
    Eigen::Vector3f v[3];
    for (int i = 0; i < 3; i++)
        v[i] = { _v[i].x(),_v[i].y(), 1.0 };
    Eigen::Vector3f f0, f1, f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);
    Eigen::Vector3f p(x, y, 1.);
    if ((p.dot(f0) * f0.dot(v[2]) > 0) && (p.dot(f1) * f1.dot(v[0]) > 0) && (p.dot(f2) * f2.dot(v[1]) > 0))
        return true;
    return false;
}

rasterizer::rasterizer(int w,int h) {
    this->w = w;
    this->h = h;
    frameBuf.resize(w * h);
    depthBuf.resize(w * h);
}

void rasterizer::SetView(const Eigen::Matrix4f &v) {
    view = v;
}

void rasterizer::SetProjection(const Eigen::Matrix4f &p) {
    projection = p;
}

void rasterizer::SetModels(const std::vector<Model> &models) {
    this->models = models;
}

void rasterizer::SetFragmentShader(const Shader &shader) {
    this->fragmentShader = shader;
}

void rasterizer::ClearDepthBuffer() {
    std::fill(depthBuf.begin(), depthBuf.end(), std::numeric_limits<float>::infinity());
}

void rasterizer::ClearColorBuffer() {
    std::fill(frameBuf.begin(), frameBuf.end(), Eigen::Vector3f{ 0, 0, 0 });
}

void rasterizer::Draw() {
    float f1 = (80 - 0.1) / 2.0;
    float f2 = (80 + 0.1) / 2.0;

    for (const auto& model : models) {
        for (const auto& t : model.triangleList) {
            Triangle& newTriangle = *t;

            Eigen::Matrix4f mvp = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f inv_trans = ( model.modelMatrix).inverse().transpose();
            mvp = Eigen::Matrix4f(projection * view * model.modelMatrix);

            std::vector<Eigen::Vector4f> tW{
                Eigen::Vector4f(model.modelMatrix * t->vertex[0]),//世界坐标提供着色
                Eigen::Vector4f(model.modelMatrix * t->vertex[1]),
                Eigen::Vector4f(model.modelMatrix * t->vertex[2])
            };
            std::vector<Eigen::Vector3f> worldPos;
            std::transform(tW.begin(), tW.end(), worldPos.begin(), [](auto& v) {
                    return v.template head<3>();
            });
            for (auto& v : newTriangle.vertex) {
                v = mvp * v;
            }
            for (auto& v : newTriangle.normal) {//法线变换
                v = inv_trans * v;
            }
            std::vector<Triangle> triangles = SutherlandHodgeman(newTriangle,tW);
            for (int i = 0; i < triangles.size(); i++) {
                std::array<Eigen::Vector3f, 3> _worldPos{worldPos[i],worldPos[i+1],worldPos[i+2]};
                RasterizeTriangle(triangles[i],_worldPos);
            }

        }
    }
}

void rasterizer::RasterizeTriangle(const Triangle &t, const std::array<Eigen::Vector3f, 3> &worldPos) {
}

