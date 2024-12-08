//
// Created by cai on 24-12-8.
//

#ifndef CAMERA_H
#define CAMERA_H
#include <iostream>
#include<Eigen/Dense>


class Camera {
public:
    Camera(const Eigen::Vector3f& position, float yaw, float pitch)
        : eye_pos(position), yaw_angle(yaw), pitch_angle(pitch)
    {
        update_camera_vectors();
        speed = 10.f;
    }

    // 设置相机位置
    void set_position(const Eigen::Vector3f& position) {
        eye_pos = position;
    }

    // 获取视图矩阵
    Eigen::Matrix4f get_view_matrix() const {
        Eigen::Vector3f target = eye_pos + front;
        return look_at(eye_pos, target, up);
    }

    // 调整 yaw 和 pitch 角度
    void rotate(float delta_yaw, float delta_pitch) {
        yaw_angle += delta_yaw;
        pitch_angle += delta_pitch;

        // 限制 pitch 角度在 -90 到 +90 度之间
        if (pitch_angle > 89.0f)
            pitch_angle = 89.0f;
        if (pitch_angle < -89.0f)
            pitch_angle = -89.0f;

        update_camera_vectors();
    }

    // 获取当前相机位置
    Eigen::Vector3f get_position() const {
        return eye_pos;
    }

    // 获取当前相机方向
    Eigen::Vector3f get_front() const {
        return front;
    }

    void Move(float delta_time , int flag) { // 0 f , 1 b ,2 l, 3 r
        float v = delta_time * speed;
        if(flag == 0) {
            eye_pos += front * v;
        }
        if(flag == 1) {
            eye_pos -= front * v;
        }
        if(flag == 2) {
            eye_pos += -right * v;
        }
        if(flag == 3) {
            eye_pos += right * v;
        }
        update_camera_vectors();
        std::cout << "Eye Position: " << eye_pos.transpose() << "\n";
        std::cout << "Front: " << front.transpose() << "\n";
        std::cout << "Right: " << right.transpose() << "\n";
    }
    void ChangePitch(float delta_time, int flag) {
        float v = delta_time * speed;
        if(flag == 0) {
            rotate(0, 5 * v);
        }
        if(flag == 1) {
            rotate(0, -5 * v);
        }
        if(flag == 2) {
            rotate(5 * v, 0);
        }
        if(flag == 3) {
            rotate(-5 * v, 0);
        }
        update_camera_vectors();
    }

    Eigen::Matrix4f look_at(const Eigen::Vector3f& eye,
                    const Eigen::Vector3f& center,
                    const Eigen::Vector3f& up_vector) const
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
private:
    Eigen::Vector3f eye_pos;
    float yaw_angle;   // 偏航角（绕Y轴旋转），单位：度
    float pitch_angle; // 俯仰角（绕X轴旋转），单位：度

    // 相机方向向量
    Eigen::Vector3f front;
    Eigen::Vector3f up;
    Eigen::Vector3f right;

    // 更新相机方向向量
    void update_camera_vectors() {
        // 将角度转换为弧度
        float yaw_rad = yaw_angle * M_PI / 180.0f;
        float pitch_rad = pitch_angle * M_PI / 180.0f;

        // 计算前方向量
        front.x() = std::cos(pitch_rad) * std::cos(yaw_rad);
        front.y() = std::sin(pitch_rad);
        front.z() = std::cos(pitch_rad) * std::sin(yaw_rad);
        front.normalize();

        // 假设世界上方向量为 (0, 1, 0)
        Eigen::Vector3f world_up(0.0f, 1.0f, 0.0f);

        // 计算右方向和上方向
        right = front.cross(world_up).normalized();
        up = right.cross(front).normalized();
    }


    // Look-At 矩阵实现
    float speed;
};




#endif //CAMERA_H
