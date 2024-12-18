#include <iostream>
#include "OBJ_Loader.h"
#include <Eigen/Dense>
#include "Model.h"
#include "rasterizer.h"
#include "Shader.h"
#include "Camera.h"
#include <chrono>
#define MY_PI 3.14159265358979323846


Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar) {
    Eigen::Matrix4f projection = Eigen::Matrix4f::Zero();

    // Convert field of view from degrees to radians
    float fov_rad = eye_fov * M_PI / 180.0f;

    // Calculate the height of the near plane
    float t = std::tan(fov_rad / 2) * zNear;
    float b = -t;

    // Calculate the width of the near plane
    float r = t * aspect_ratio;
    float l = -r;

    // Define the projection matrix components
    projection(0, 0) = 2 * zNear / (r - l);
    projection(1, 1) = 2 * zNear / (t - b);
    projection(0, 2) = (r + l) / (r - l);
    projection(1, 2) = (t + b) / (t - b);
    projection(2, 2) = -(zFar + zNear) / (zFar - zNear);
    projection(2, 3) = -2 * zFar * zNear / (zFar - zNear);
    projection(3, 2) = -1;


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

Eigen::Matrix4f look_at(const Eigen::Vector3f& eye,
                const Eigen::Vector3f& center,
                const Eigen::Vector3f& up_vector)
{
    Eigen::Vector3f f = (center - eye).normalized();
    Eigen::Vector3f s = f.cross(up_vector).normalized();
    Eigen::Vector3f u = s.cross(f).normalized();

    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat(0, 0) = s.x();
    mat(0, 1) = s.y();
    mat(0, 2) = s.z();
    mat(1, 0) = u.x();
    mat(1, 1) = u.y();
    mat(1, 2) = u.z();
    mat(2, 0) = -f.x();
    mat(2, 1) = -f.y();
    mat(2, 2) = -f.z();
    mat(0, 3) = -s.dot(eye);
    mat(1, 3) = -u.dot(eye);
    mat(2, 3) = f.dot(eye);

    return mat;
}


int main()
{
    using Clock = std::chrono::high_resolution_clock;
    std::string TexturePath1 = "/Users/yunicai/Code_file/code/GAMES101-Homework/Assignment3/Assignment3/Code/models/spot/spot_texture.png";
    std::string ModelPath1 = "/Users/yunicai/Code_file/code/GAMES101-Homework/Assignment3/Assignment3/Code/models/spot/spot_triangulated_good.obj";
    std::string ModelPath2 = "/Users/yunicai/Code_file/code/GAMES101-Homework/Assignment3/Assignment3/Code/models/bunny/bunny.obj";
    std::string ModelPath3 = "/Users/yunicai/Code_file/code/GAMES101-Homework/Assignment3/Assignment3/Code/models/rock/rock.obj";
    std::string ModelPath7 = "/Users/yunicai/Model/blue-archive-sunohara-kokona/cocona.obj";
    std::string TexturePath3 = "/Users/yunicai/Code_file/code/GAMES101-Homework/Assignment3/Assignment3/Code/models/rock/rock.png";
    std::string TexturePath2 = "/Users/yunicai/Code_file/code/computer_graphic_homework/Raster/image/img.png";
    Model model1;
    Texture texture1(TexturePath1);
    model1.SetTexture(texture1);
    model1.LoadModel(ModelPath1);
    model1.SetRotation(Eigen::Vector3f(0.f, 1.f, 0.f),30);
    model1.SetPosition(0, 0, 0.f);
    model1.SetScale(Eigen::Vector3f(5, 5, 5));
    model1.GetModelMatrix();


    Model model3;
    Texture texture3(TexturePath3);
    model3.SetTexture(texture3);
    model3.LoadModel(ModelPath3);

    model3.SetRotation(Eigen::Vector3f(0.f, 1.f, 0.f),30);
    model3.SetPosition(-10.f, -10.f, 1.f);
    model3.SetScale(Eigen::Vector3f(4, 4, 4));
    model3.GetModelMatrix();

    Model model2;
    Texture texture2(TexturePath2);
    model2.SetTexture(texture3);
    model2.LoadModel(ModelPath2);

    model2.SetRotation(Eigen::Vector3f(0.f, -10.f, 0.f),0);
    model2.SetScale(Eigen::Vector3f(20, 20, 20));
    model2.SetPosition(0.f, -5.f, -5.f);
    model2.SetTexture(texture2);
    model2.GetModelMatrix();

    Model model4;
    Texture texture4(TexturePath3);
    model4.SetTexture(texture3);
    model4.LoadModel(ModelPath3);
    // model4.SetTexture(texture3);
    model4.SetRotation(Eigen::Vector3f(0.f, 1.f, 0.f),30);
    model4.SetPosition(10.f, -10.f, 1.f);
    model4.SetScale(Eigen::Vector3f(4, 4, 4));
    model4.GetModelMatrix();

    Model model5;
    model5.SetTexture(texture3);
    model5.LoadModel(ModelPath3);
    // model5.SetTexture(texture3);
    model5.SetRotation(Eigen::Vector3f(0.f, 1.f, 0.f),30);
    model5.SetPosition(0.f, -10.f, 5.f);
    model5.SetScale(Eigen::Vector3f(4, 4, 4));
    model5.GetModelMatrix();

    Model model6;
    model6.SetTexture(texture3);
    model6.LoadModel(ModelPath3);
    model6.SetRotation(Eigen::Vector3f(0.f, -5.f, 0.f),30);
    model6.SetPosition(0.f, -10.f, -10.f);
    model6.SetScale(Eigen::Vector3f(4, 4, 4));
    model6.GetModelMatrix();

    Model model7;
    model7.LoadModel(ModelPath7);
    // model7.SetTexture(texture2);
    model7.SetRotation(Eigen::Vector3f(0.f, -5.f, 0.f),-90);
    model7.SetPosition(0.f, -3.f, 10.f);
    model7.SetScale(Eigen::Vector3f(6, 6, 6));
    model7.GetModelMatrix();

    std::vector<Model> models;
    models.push_back(model1);
    models.push_back(model2);
    models.push_back(model3);
    models.push_back(model4);
    models.push_back(model5);
    models.push_back(model6);
    models.push_back(model7);
    //模型信息初始化

    Eigen::Vector3f eye_pos(0, 0, 20);
    Shader shader;

    Light asLight;
    asLight.intensity << 0.6,0.6,0.6;
    asLight.position << -20, 10 ,0 ;
    Light light;
    light.intensity << 4 ,4 ,4;
    light.position << 20, 20,0;
    shader.SetLight(light);
    shader.SetLight(asLight);
    //着色器初始化

    Eigen::Matrix4f lightView = look_at(light.position,Eigen::Vector3f(0.f,0.f,0.f),Eigen::Vector3f(0.f,1.f,0.f));
    Eigen::Matrix4f lightProj = get_projection_matrix(45,1,0.1,80);
    Eigen::Matrix4f lightViewProj =  lightProj * lightView ;
    rasterizer shadowRst(700,700);
    shadowRst.SetModels(models);
    shadowRst.SetView(lightView);
    shadowRst.SetProjection(lightProj);
    shadowRst.SetFragmentShader(shader);
    shadowRst.ClearColorBuffer();
    shadowRst.ClearDepthBuffer();
    shadowRst.Draw();
    cv::Mat simage(700, 700, CV_32FC3, shadowRst.frame_buffer().data());
    simage.convertTo(simage, CV_8UC3, 1.f);
    cv::cvtColor(simage, simage, cv::COLOR_RGB2BGR);
    cv::imwrite("shadow.png", simage);
    std::vector<float> shadowMap = shadowRst.depth_buffer();



    //光源视角

    Camera camera(eye_pos, -90, 0);
    std::cout << camera.get_view_matrix() << std::endl;


    rasterizer rst(700,700);
    rst.SetModels(models);
    rst.SetProjection(get_projection_matrix(45.0, 1, 0.1, 80));
    rst.SetFragmentShader(shader);
    rst.SetView(get_view_matrix(eye_pos));
    // rst.Draw();


    // rst.SetView(lightView );
    //渲染器初始化

    auto lastFrame = Clock::now();
    float deltaTime = 0.0f;
    //这里是为了计算帧时间

    int key = 0;
    std::string filename = "test.jpg";
    while(key != 27)
    {
        auto currentFrame = Clock::now();

        // 计算 deltaTime，以秒为单位
        std::chrono::duration<float> deltaTimeDuration = currentFrame - lastFrame;
        deltaTime = deltaTimeDuration.count();


        rst.ClearColorBuffer();
        rst.ClearDepthBuffer();
        rst.SetView(camera.get_view_matrix());
        // rst.SetView(lightView);
        rst.SetProjection(get_projection_matrix(45.0, 1, 0.1, 80));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        rst.DrawWithShadow(lightViewProj, shadowMap);
        // rst.Draw();
        cv::Mat image(700, 700, CV_32FC3, rst.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(1);

        if (key == 'w')
            camera.Move(deltaTime, 0);
        if (key == 's')
            camera.Move(deltaTime, 1);
        if (key == 'a')
            camera.Move(deltaTime, 2);
        if (key == 'd')
            camera.Move(deltaTime, 3);
        if (key == 'l')
            camera.ChangePitch(deltaTime,2);
        if (key == 'j')
            camera.ChangePitch(deltaTime,3);
        if (key == 'k')
            camera.ChangePitch(deltaTime,1);
        if (key == 'i')
            camera.ChangePitch(deltaTime,0);

        lastFrame = currentFrame;

    }






}
