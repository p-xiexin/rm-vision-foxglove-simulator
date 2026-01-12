#include <Eigen/Dense>
#include <iostream>

#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 1024

class Camera {
public:
    // 构造函数，初始化摄像头外参和内参，注意 +x 指向图像的右侧，+y 指向下方，+z 指向图像的平面。
    Camera(const Eigen::Matrix3d& intrinsic_matrix, const Eigen::Matrix3d& Rbc, const Eigen::Vector3d& tbc)
        : intrinsic_matrix_(intrinsic_matrix), R_bc_(Rbc), t_bc_(tbc) {
        // R_bc_ = extrinsic_rotation_.transpose();
        // t_bc_ = -extrinsic_rotation_.transpose() * extrinsic_translation_;
        extrinsic_rotation_ = R_bc_.transpose();
        extrinsic_translation_ = -R_bc_.transpose() * t_bc_;
    }

    // 获取相机内参
    Eigen::Matrix3d getIntrinsicMatrix() const {
        return intrinsic_matrix_;
    }

    // 获取相机外参旋转矩阵
    Eigen::Matrix3d getExtrinsicRotation() const {
        return extrinsic_rotation_;
    }

    // 获取相机外参平移向量
    Eigen::Vector3d getExtrinsicTranslation() const {
        return extrinsic_translation_;
    }

    // 获取畸变参数
    std::vector<double> getDistortionParams() const {
        return distortion_params_;
    }

    // 设置畸变参数
    void setDistortionParams(const std::vector<double>& distortion_params) {
        distortion_params_ = distortion_params;
    }

    // 获取相机位姿
    std::pair<Eigen::Quaterniond, Eigen::Vector3d> getCameraPose() const {
        return std::make_pair(Eigen::Quaterniond(R_bc_), t_bc_);
    }

    // 设置相机在真实世界实体的尺寸（以米为单位）
    void setPhysicalSize(const Eigen::Vector3d& size) {
        physical_size_ = size;
    }

    // 获取相机在真实世界中的尺寸
    Eigen::Vector3d getPhysicalSize() const {
        return physical_size_;
    }

    // 将世界坐标系下的点转换为相机像素坐标系下的点
    std::vector<Eigen::Vector2d> worldToPixel(const std::vector<Eigen::Vector3d>& world_points, Eigen::Matrix3d R_wb, Eigen::Vector3d t_wb) const {
        std::vector<Eigen::Vector2d> pixel_points;

        for (const auto& world_point : world_points) {
            Eigen::Vector3d body_point = R_wb.transpose() * (world_point - t_wb); // 将点从世界坐标系转换到机体坐标系
            Eigen::Vector3d camera_point = extrinsic_rotation_ * body_point + extrinsic_translation_; // 将点从机体坐标系转换到相机坐标系
            Eigen::Vector3d pixel_point = intrinsic_matrix_ * camera_point; // 将点从相机坐标系转换到像素坐标系

            // 归一化像素坐标
            Eigen::Vector2d normalized_pixel_point = pixel_point.hnormalized();

            // 判断像素坐标是否在范围内
            if (normalized_pixel_point.x() >= 0 && normalized_pixel_point.x() < IMAGE_WIDTH &&
                normalized_pixel_point.y() >= 0 && normalized_pixel_point.y() < IMAGE_HEIGHT) {
                pixel_points.push_back(normalized_pixel_point);
            } else {
                // 如果像素坐标超出范围，则将特殊值(-1, -1)添加到结果中表示无效
                pixel_points.push_back(Eigen::Vector2d(-1, -1));
            }
        }

        return pixel_points;
    }

    // 输出相机内外参
    void printCameraParams() const {
        std::cout << "Camera Intrinsics:" << std::endl << intrinsic_matrix_ << std::endl;
        std::cout << "Camera Extrinsics (Rotation):" << std::endl << extrinsic_rotation_ << std::endl;
        std::cout << "Camera Extrinsics (Translation):" << std::endl << extrinsic_translation_ << std::endl;
        std::cout << "Distortion Parameters:" << std::endl;
        for (const auto& param : distortion_params_) {
            std::cout << param << " ";
        }
        std::cout << std::endl;
    }

private:
    // 应用畸变参数
    void applyDistortion(Eigen::Vector3d& point) const {
        double k1 = distortion_params_[0];
        double k2 = distortion_params_[1];
        double p1 = distortion_params_[2];
        double p2 = distortion_params_[3];

        double x = point.x();
        double y = point.y();
        double r2 = x * x + y * y;
        double r4 = r2 * r2;

        double x_distorted = x * (1 + k1 * r2 + k2 * r4) + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
        double y_distorted = y * (1 + k1 * r2 + k2 * r4) + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

        point.x() = x_distorted;
        point.y() = y_distorted;
    }

private:
    Eigen::Matrix3d intrinsic_matrix_; // 摄像头内参
    // 摄像头外参，为机体坐标系到相机姿态的逆变换
    Eigen::Matrix3d extrinsic_rotation_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d extrinsic_translation_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_bc_ = Eigen::Matrix3d::Identity(); // 从机体坐标系到相机坐标系的变换，相机的姿态
    Eigen::Vector3d t_bc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d physical_size_ = Eigen::Vector3d::Zero(); // 相机在真实世界中的尺寸
    std::vector<double> distortion_params_; // 畸变参数
};
