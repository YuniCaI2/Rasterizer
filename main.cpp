#include <iostream>
#include "OBJ_Loader.h"
#include <Eigen/Dense>
#include "Model.h"
#include "rasterizer.h"
#include "Shader.h"
#define MY_PI 3.14159265358979323846

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();//这里必须单位化，否则黑屏

    eye_fov = eye_fov * MY_PI / 180;
    float fax = 1.0f / (float)tan(eye_fov * 0.5f);

    Eigen::Matrix4f perspective;
    perspective << (float)(fax / aspect_ratio),0,0,0,
                    0,(float)(fax),0,0,
                    0,0,zFar / (zFar - zNear),0,
                    0,0,1,-zNear * zFar / (zFar - zNear);
    Eigen::Matrix4f mirror;
    mirror <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;

    projection = mirror * perspective * projection;

    return projection;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

int main()
{
    std::string TexturePath1 = "/Users/yunicai/Code_file/code/GAMES101-Homework/Assignment3/Assignment3/Code/models/spot/spot_texture.png";
    std::string ModelPath1 = "/Users/yunicai/Code_file/code/GAMES101-Homework/Assignment3/Assignment3/Code/models/spot/spot_triangulated_good.obj";
    Model model1;
    Texture texture1(TexturePath1);
    model1.LoadModel(ModelPath1);
    model1.SetTexture(texture1);
    model1.SetRotation(Eigen::Vector3f(0.f, 1.f, 0.f),30);
    model1.SetPosition(0, 0, 5.f);
    model1.SetScale(Eigen::Vector3f(5, 5, 5));
    model1.GetModelMatrix();
    std::vector<Model> models;
    models.push_back(model1);
    //模型信息初始化

    Eigen::Vector3f eye_pos(0, 0, 25);
    Shader shader;

    Light asLight;
    asLight.intensity << 0.6,0.6,0.6;
    asLight.position << -20, 10 ,0 ;
    Light light;
    light.intensity << 1.5 ,1.5 ,1.5;
    light.position << 20, 20,0;
    shader.SetLight(light);
    shader.SetLight(asLight);
    //着色器初始化

    rasterizer rst(700,700);
    rst.SetModels(models);
    rst.SetProjection(get_projection_matrix(45.0, 1, 0.1, 80));
    rst.SetFragmentShader(shader);
    rst.SetView(get_view_matrix(eye_pos));
    //渲染器初始化

    int key = 0;
    std::string filename = "test.jpg";
    while(key != 27)
    {
        rst.ClearColorBuffer();
        rst.ClearDepthBuffer();

        rst.SetProjection(get_projection_matrix(45.0, 1, 0.1, 80));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        rst.Draw();
        cv::Mat image(700, 700, CV_32FC3, rst.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

    }






}
