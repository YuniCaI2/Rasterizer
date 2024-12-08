//
// Created by cai on 24-12-6.
//

#include "rasterizer.h"

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Eigen::Vector4f* v) {
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return { c1,c2,c3 };
}

static auto interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma,
                                   const Eigen::Vector2f& vert1,
                                   const Eigen::Vector2f& vert2,
                                   const Eigen::Vector2f& vert3,
                                   float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static const std::vector<Eigen::Vector4f> ViewLines = {
    Eigen::Vector4f(0,0,-1,1),
    Eigen::Vector4f(0,0,1,1),
    Eigen::Vector4f(1,0,0,1),
    Eigen::Vector4f(-1,0,0,1),
    Eigen::Vector4f(0,1,0,1),
    Eigen::Vector4f(0,-1,0,1)

};

 static bool Inside(const Eigen::Vector4f &line,const Eigen::Vector4f &p) {
    return line.x() * p.x() + line.y() * p.y() + line.z() * p.z() + line.w() * p.w() < 0;
}

static float IntersectWeight(const Eigen::Vector4f &line, const Eigen::Vector4f& v1, const Eigen::Vector4f& v2) {
     float d1 = line.x() * v1.x() + line.y() * v1.y() + line.z() * v1.z() + line.w() * v1.w();
     float d2 = line.x() * v2.x() + line.y() * v2.y() + line.z() * v2.z() + line.w() * v2.w();

     // 避免除以零
     if (fabs(d1 - d2) < 1e-6) {
         return 0.0f; // 或者其他合适的默认值
     }

     return d1 / (d1 - d2);
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
        check[i] = Inside(left, vertex) && Inside(right, vertex) && Inside(bottom, vertex)
           && Inside(top, vertex) && Inside(far, vertex) && Inside(near, vertex);
        i++;
    }
    return check[0] && check[1] && check[2];
}


std::vector<Triangle> SutherlandHodgeman(const Triangle& triangle, std::vector<Eigen::Vector3f>& worldPoses) {
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
    for(int i = 1; i < resultVertices.size() - 1; i++) {
        Triangle tempTriangle;
        tempTriangle.setVertexs(std::array<Eigen::Vector4f, 3>{resultVertices[0].position,resultVertices[i].position,resultVertices[i+1].position});
        tempTriangle.setNormals(std::array<Eigen::Vector3f, 3>{resultVertices[0].normal,resultVertices[i].normal,resultVertices[i+1].normal});
        tempTriangle.setTexCoords(std::array<Eigen::Vector2f, 3>{resultVertices[0].texcoord,resultVertices[i].texcoord,resultVertices[i+1].texcoord});
        tempTriangle.setColors(std::array<Eigen::Vector3f, 3>{resultVertices[0].color,resultVertices[i].color,resultVertices[i+1].color});
        worldPoses.emplace_back(resultVertices[0].worldPos);
        worldPoses.emplace_back(resultVertices[i].worldPos);
        worldPoses.emplace_back(resultVertices[i+1].worldPos);
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
     texture = nullptr;
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

    for (auto& model : models) {
        for (const auto& t : model.triangleList) {
            Triangle newTriangle = *t;
            texture = &model.texture;
            Eigen::Matrix4f mvp = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f inv_trans = ( model.modelMatrix).inverse().transpose();
            mvp = Eigen::Matrix4f(projection * view * model.modelMatrix);
            // std::cout << "m:" << std::endl;
            // std::cout << model.modelMatrix << std::endl;
            // std::cout << "v" << std::endl;
            // std::cout << view << std::endl;
            // std::cout << "p" << std::endl;
            // std::cout << projection << std::endl;


            std::vector<Eigen::Vector4f> tW{
                Eigen::Vector4f(model.modelMatrix * t->vertex[0]),//世界坐标提供着色
                Eigen::Vector4f(model.modelMatrix * t->vertex[1]),
                Eigen::Vector4f(model.modelMatrix * t->vertex[2])
            };
            std::vector<Eigen::Vector3f> worldPos(tW.size());
            std::transform(tW.begin(), tW.end(), worldPos.begin(), [](const Eigen::Vector4f& v) {
                return v.head<3>();
            });
            // std::cout << "Triangle:" << std::endl;
            // std::cout << newTriangle.vertex[0] << " " << newTriangle.vertex[1] << " " << newTriangle.vertex[2] << std::endl;
            for (auto& v : newTriangle.vertex) {
                v = mvp * v;
            }
            // std::cout << "MVP:" << std::endl;
            // std::cout << mvp << std::endl;

            for (auto& v : newTriangle.normal) {//法线变换
                Eigen::Vector4f t_v = Eigen::Vector4f(v[0], v[1], v[2], 1);
                v = (inv_trans * t_v).head<3>();
            }
            std::vector<Triangle> triangles = SutherlandHodgeman(newTriangle,worldPos);
            // std::vector<Triangle> triangles;
            // triangles.push_back(newTriangle);
            // for(auto& vert : newTriangle.vertex) {
            //     vert.x() = 0.5 * w * (vert.x() + 1.0);
            //     vert.y() = 0.5 * h * (vert.y() + 1.0);
            //     vert.z() = vert.z() * f1 + f2;
            // }
            for (auto& triangle : triangles) {
                for (auto& v : triangle.vertex) {
                    v = v / v.w();
                }//齐次
            }
            // std::cout<< "Triangle is :" << triangles[0].vertex[0] << std::endl;
            for (int i = 0; i < triangles.size(); i++) {
                for (int j = 0; j < 3; j++) {
                    triangles[i].vertex[j].x() = 0.5f * w * (triangles[i].vertex[j].x() + 1.0f);
                    triangles[i].vertex[j].y() = 0.5f * h * (1.0f + triangles[i].vertex[j].y());
                    triangles[i].vertex[j].z() = triangles[i].vertex[j].z() * f1 + f2;
                    std::cout << "Z:" << triangles[i].vertex[j].z() << std::endl;
                }
                std::array<Eigen::Vector3f, 3> _worldPos{worldPos[i*3],worldPos[i*3+1],worldPos[i*3+2]};

                RasterizeTriangle(triangles[i],_worldPos);
            }

        }
    }
}

void rasterizer::RasterizeTriangle(const Triangle &t, const std::array<Eigen::Vector3f, 3> &worldPos) {
    std::array<Eigen::Vector4f, 3> v{t.vertex[0], t.vertex[1], t.vertex[2]};

    float max_x, min_x, max_y, min_y;

    max_x = v[0].x() > v[1].x() ? v[0].x() : v[1].x();
    max_x = max_x > v[2].x() ? max_x : v[2].x();

    min_x = v[0].x() < v[1].x() ? v[0].x() : v[1].x();
    min_x = min_x < v[2].x() ? min_x : v[2].x();

    max_y = v[0].y() > v[1].y() ? v[0].y() : v[1].y();
    max_y = max_y > v[2].y() ? max_y : v[2].y();

    min_y = v[0].y() < v[1].y() ? v[0].y() : v[1].y();
    min_y = min_y < v[2].y() ? min_y : v[2].y();

     // std::cout << "Triangle: " << max_y << " "<< max_x << std::endl;
    for (int i = min_x; i < max_x; i++) {
        for (int j = min_y; j < max_y; j++) {
            // std::cout << t.vertex[i].z() << std::endl;
            if (InsideTriangle(i, j, t.vertex) == true) {
                auto [alpha, beta, gamma] = computeBarycentric2D(i, j, t.vertex);
                float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                zp *= Z;
                int index = GetIndex(i, j);
                if (zp <= this->depthBuf[index]) {
                    auto interpolated_color = interpolate(alpha, beta, gamma, t.color[0], t.color[1], t.color[2], 1); //interpolate color
                    auto interpolated_normal = interpolate(alpha, beta, gamma, t.normal[0], t.normal[1], t.normal[2], 1); //interpolate normal
                    auto interpolated_texcoords = interpolate(alpha, beta, gamma, t.tex[0], t.tex[1], t.tex[2], 1); //interpolate texture
                    auto interpolated_shadingcoords = interpolate(alpha, beta, gamma, worldPos[0], worldPos[1], worldPos[2], 1); //interpolate viewpos
                    this->depthBuf[index] = zp; //update zbuffer
                    fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
                    payload.world_pos = interpolated_shadingcoords;
                    auto pixel_color = fragmentShader.UsingShader(payload);
                    // std::cout << pixel_color << std::endl;
                    SetPixel(Eigen::Vector2i(700 - i, 700 - j), pixel_color);
                }

            }

        }
    }

}

int rasterizer::GetIndex(int x, int y) {
    return (h - 1 - y) * w + x;
}

void rasterizer::SetPixel(const Eigen::Vector2i &point, const Eigen::Vector3f &color) {
     //old index: auto ind = point.y() + point.x() * width;
     int ind = (h - point.y()) * w + point.x();
     frameBuf[ind] = color;
}
