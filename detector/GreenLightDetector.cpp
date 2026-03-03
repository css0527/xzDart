#include "GreenLightDetector.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <memory>
#include <algorithm>
#include <iomanip>
#include <stdexcept>
#include <cmath>
#include <random>

// 确保先包含协议处理器头文件
#include "../io/serial_cboard.hpp"
#include "../tools/yaml.hpp"

using namespace std;
using namespace cv;

// 常数定义
const float DEG_TO_RAD = CV_PI / 180.0f;
const float RAD_TO_DEG = 180.0f / CV_PI;

// GreenLightDetector 实现
GreenLightDetector::GreenLightDetector() 
    : kf_initialized(false)
    , valid_frames_count(0) {
}

GreenLightDetector::~GreenLightDetector() {
}

bool GreenLightDetector::loadCameraParameters() {
    try {
        auto config_yaml = tools::load(config_path);
        
        // 读取相机内参矩阵 [3x3]
        if (auto camera_matrix_node = tools::read_optional<std::vector<double>>(config_yaml, "camera_matrix")) {
            std::vector<double> matrix_data = *camera_matrix_node;
            if (matrix_data.size() == 9) {
                config.camera_matrix = (Mat_<double>(3, 3) << 
                    matrix_data[0], matrix_data[1], matrix_data[2],
                    matrix_data[3], matrix_data[4], matrix_data[5],
                    matrix_data[6], matrix_data[7], matrix_data[8]);
                
                cout << "Camera matrix loaded:" << endl;
                cout << config.camera_matrix << endl;
            } else {
                cerr << "Camera matrix should have 9 elements" << endl;
                return false;
            }
        } else {
            cerr << "Camera matrix not found in config" << endl;
            return false;
        }
        
        // 读取畸变系数 [5x1]
        if (auto dist_coeffs_node = tools::read_optional<std::vector<double>>(config_yaml, "distort_coeffs")) {
            std::vector<double> dist_data = *dist_coeffs_node;
            if (dist_data.size() >= 5) {
                config.dist_coeffs = Mat::zeros(5, 1, CV_64F);
                for (size_t i = 0; i < 5; ++i) {
                    config.dist_coeffs.at<double>(i) = dist_data[i];
                }
                
                cout << "Distortion coefficients loaded:" << endl;
                cout << config.dist_coeffs << endl;
            } else {
                cerr << "Distortion coefficients should have at least 5 elements" << endl;
                return false;
            }
        } else {
            cerr << "Distortion coefficients not found in config" << endl;
            return false;
        }
        
        // 读取相机-云台旋转矩阵 R_camera2gimbal [3x3]
        if (auto R_node = tools::read_optional<std::vector<double>>(config_yaml, "R_camera2gimbal")) {
            std::vector<double> R_data = *R_node;
            if (R_data.size() == 9) {
                config.R_camera2gimbal = (Mat_<double>(3, 3) << 
                    R_data[0], R_data[1], R_data[2],
                    R_data[3], R_data[4], R_data[5],
                    R_data[6], R_data[7], R_data[8]);
                
                cout << "Camera to gimbal rotation matrix loaded:" << endl;
                cout << config.R_camera2gimbal << endl;
            } else {
                cerr << "R_camera2gimbal should have 9 elements" << endl;
                return false;
            }
        } else {
            cerr << "R_camera2gimbal not found in config" << endl;
            return false;
        }
        
        // 读取相机-云台平移向量 t_camera2gimbal [3]
        if (auto t_node = tools::read_optional<std::vector<double>>(config_yaml, "t_camera2gimbal")) {
            std::vector<double> t_data = *t_node;
            if (t_data.size() == 3) {
                config.t_camera2gimbal = (Mat_<double>(3, 1) << 
                    t_data[0], t_data[1], t_data[2]);
                
                cout << "Camera to gimbal translation vector loaded:" << endl;
                cout << config.t_camera2gimbal << endl;
            } else {
                cerr << "t_camera2gimbal should have 3 elements" << endl;
                return false;
            }
        } else {
            cerr << "t_camera2gimbal not found in config" << endl;
            return false;
        }
        
        // 设置目标半径（根据配置文件中的直径）
        config.target_radius_3d = config.real_diameter_mm / 2000.0f; // 转换为米
        
        return true;
    }
    catch (const std::exception& e) {
        cerr << "Error loading camera parameters from YAML: " << e.what() << endl;
        return false;
    }
}

bool GreenLightDetector::loadConfig(const std::string& config_path) {
    try {
        auto config_yaml = tools::load(config_path);
        
        // Camera Configuration
        if (auto val = tools::read_optional<std::string>(config_yaml, "camera_name")) config.camera_name = *val;
        if (auto val = tools::read_optional<float>(config_yaml, "exposure_ms")) config.exposure_ms = *val;
        if (auto val = tools::read_optional<float>(config_yaml, "gain")) config.gain = *val;
        if (auto val = tools::read_optional<int>(config_yaml, "width")) config.width = *val;
        if (auto val = tools::read_optional<int>(config_yaml, "height")) config.height = *val;
        if (auto val = tools::read_optional<int>(config_yaml, "fps")) config.fps = *val;
        if (auto val = tools::read_optional<std::string>(config_yaml, "vid_pid")) config.vid_pid = *val;
        
        // Image Processing Configuration
        if (auto val = tools::read_optional<int>(config_yaml, "h_min")) config.h_min = *val;
        if (auto val = tools::read_optional<int>(config_yaml, "h_max")) config.h_max = *val;
        if (auto val = tools::read_optional<int>(config_yaml, "s_min")) config.s_min = *val;
        if (auto val = tools::read_optional<int>(config_yaml, "s_max")) config.s_max = *val;
        if (auto val = tools::read_optional<int>(config_yaml, "v_min")) config.v_min = *val;
        if (auto val = tools::read_optional<int>(config_yaml, "v_max")) config.v_max = *val;
        if (auto val = tools::read_optional<int>(config_yaml, "morph_open_size")) config.morph_open_size = *val;
        if (auto val = tools::read_optional<int>(config_yaml, "morph_close_size")) config.morph_close_size = *val;
        if (auto val = tools::read_optional<float>(config_yaml, "min_circularity")) config.min_circularity = *val;
        if (auto val = tools::read_optional<int>(config_yaml, "min_area")) config.min_area = *val;
        if (auto val = tools::read_optional<int>(config_yaml, "max_area")) config.max_area = *val;
        
        // Distance Calculation
        if (auto val = tools::read_optional<float>(config_yaml, "real_diameter_mm")) config.real_diameter_mm = *val;
        if (auto val = tools::read_optional<float>(config_yaml, "focal_length_pixels")) config.focal_length_pixels = *val;
        
        // Serial Port Configuration
        if (auto val = tools::read_optional<std::string>(config_yaml, "serial_port")) config.serial_port = *val;
        if (auto val = tools::read_optional<int>(config_yaml, "serial_baudrate")) config.serial_baudrate = *val;
        if (auto val = tools::read_optional<std::string>(config_yaml, "serial_data_format")) config.serial_data_format = *val;
        
        // Display Configuration
        if (auto val = tools::read_optional<bool>(config_yaml, "show_preview")) config.show_preview = *val;
        if (auto val = tools::read_optional<bool>(config_yaml, "draw_bounding_box")) config.draw_bounding_box = *val;
        if (auto val = tools::read_optional<bool>(config_yaml, "draw_center")) config.draw_center = *val;
        if (auto val = tools::read_optional<bool>(config_yaml, "show_distance")) config.show_distance = *val;
        if (auto val = tools::read_optional<bool>(config_yaml, "show_mask")) config.show_mask = *val;
        if (auto val = tools::read_optional<float>(config_yaml, "window_scale")) config.window_scale = *val;
        
        // 鲁棒性配置
        if (auto val = tools::read_optional<int>(config_yaml, "smoothing_frames")) config.smoothing_frames = *val;
        else config.smoothing_frames = 3;  // 默认值
        
        if (auto val = tools::read_optional<float>(config_yaml, "gaussian_blur_sigma")) config.gaussian_blur_sigma = *val;
        else config.gaussian_blur_sigma = 1.0f;  // 默认值
        
        if (auto val = tools::read_optional<float>(config_yaml, "bilateral_filter_sigma")) config.bilateral_filter_sigma = *val;
        else config.bilateral_filter_sigma = 75.0f;  // 默认值
        
        if (auto val = tools::read_optional<bool>(config_yaml, "enable_kalman_filter")) config.enable_kalman_filter = *val;
        else config.enable_kalman_filter = true;  // 默认启用
        
        if (auto val = tools::read_optional<int>(config_yaml, "min_consecutive_frames")) config.min_consecutive_frames = *val;
        else config.min_consecutive_frames = 2;  // 默认值
        
        // 默认显示PNP结果
        config.show_pnp_results = true;
        
        // 电机配置 - 设置默认值
        config.motor_distance_per_rotation_mm = 10.0f;
        config.motor_max_rotations = 5.0f;
        config.motor_min_rotations = -5.0f;
        config.force_motor_rotations = false;
        config.default_motor_rotations = 0.0f;
        
        // 从 YAML 读取电机配置（支持 bool 和 float 混合）
        if (config_yaml["motor_config"]) {
            auto motor_node = config_yaml["motor_config"];
            
            if (motor_node["distance_per_rotation_mm"]) {
                config.motor_distance_per_rotation_mm = motor_node["distance_per_rotation_mm"].as<float>();
            }
            
            if (motor_node["max_rotations"]) {
                config.motor_max_rotations = motor_node["max_rotations"].as<float>();
            }
            
            if (motor_node["min_rotations"]) {
                config.motor_min_rotations = motor_node["min_rotations"].as<float>();
            }
            
            // 读取 bool 类型的 force_motor_rotations
            if (motor_node["force_motor_rotations"]) {
                config.force_motor_rotations = motor_node["force_motor_rotations"].as<bool>();
            }
            
            // 读取 float 类型的 default_motor_rotations
            if (motor_node["default_motor_rotations"]) {
                config.default_motor_rotations = motor_node["default_motor_rotations"].as<float>();
            }
        }

        // 飞镖检测策略
        if (config_yaml["dart_strategy"]) {
            std::string strat = config_yaml["dart_strategy"].as<std::string>();
            if (strat == "fixed") config.dart_strategy = Config::DartStrategy::Fixed;
            else if (strat == "random_fixed") config.dart_strategy = Config::DartStrategy::RandomFixed;
        }
        if (config_yaml["random_move_range_degrees"]) {
            config.random_move_range_degrees = config_yaml["random_move_range_degrees"].as<float>();
        }
        if (config_yaml["random_move_after_hit"]) {
            config.random_move_after_hit = config_yaml["random_move_after_hit"].as<bool>();
        }
        
        cout << "Motor config loaded: distance_per_rotation=" << config.motor_distance_per_rotation_mm 
             << "mm, max_rotations=" << config.motor_max_rotations 
             << ", min_rotations=" << config.motor_min_rotations << endl;
        
        cout << "Configuration loaded from " << config_path << endl;
        return true;
    }
    catch (const std::exception& e) {
        cerr << "Error loading config file: " << e.what() << endl;
        return false;
    }
}

bool GreenLightDetector::initialize(const std::string& config_path) {
    this->config_path = config_path;
    
    // 加载配置
    if (!loadConfig(config_path)) {
        return false;
    }
    
    // 加载相机参数（内参和相机-云台转换）
    if (!loadCameraParameters()) {
        cerr << "Failed to load camera parameters" << endl;
        return false;
    }
    
    // 初始化串口（使用实际的 SerialBoard）
    try {
        serial_board_ = std::make_unique<io::SerialBoard>(config_path);
        cout << "Serial board initialized successfully" << endl;
    } catch (const std::exception& e) {
        cerr << "Warning: Serial board initialization failed: " << e.what() << endl;
        cerr << "Continuing without serial communication..." << endl;
        serial_board_ = nullptr;
    }

    // dart 模块初始状态
    initial_motor_rotations = 0.0;
    current_motor_rotations = initial_motor_rotations;
    detection_window_active = false;
    rng.seed(std::random_device{}());
    
    return true;
}

void GreenLightDetector::initializeKalmanFilter() {
    if (kf_initialized) return;
    
    // 初始化中心点卡尔曼滤波器 (2D: x, y 坐标)
    kf_center.init(4, 2, 0);  // 4个状态变量, 2个测量值
    
    // 状态转移矩阵 [x, y, vx, vy]
    setIdentity(kf_center.transitionMatrix);
    kf_center.transitionMatrix.at<float>(0, 2) = 1.0f;  // x += vx
    kf_center.transitionMatrix.at<float>(1, 3) = 1.0f;  // y += vy
    
    // 测量矩阵 [x, y]
    setIdentity(kf_center.measurementMatrix, Scalar::all(1));
    kf_center.measurementMatrix.resize(2, 4);
    
    // 过程噪声协方差
    setIdentity(kf_center.processNoiseCov, Scalar::all(1e-4));
    
    // 测量噪声协方差
    setIdentity(kf_center.measurementNoiseCov, Scalar::all(1e-1));
    
    // 初始化半径卡尔曼滤波器 (1D: radius)
    kf_radius.init(2, 1, 0);  // 2个状态变量, 1个测量值
    
    // 状态转移矩阵 [r, v_r]
    setIdentity(kf_radius.transitionMatrix);
    kf_radius.transitionMatrix.at<float>(0, 1) = 1.0f;  // r += v_r
    
    // 测量矩阵 [r]
    setIdentity(kf_radius.measurementMatrix, Scalar::all(1));
    kf_radius.measurementMatrix.resize(1, 2);
    
    // 过程噪声协方差
    setIdentity(kf_radius.processNoiseCov, Scalar::all(1e-4));
    
    // 测量噪声协方差
    setIdentity(kf_radius.measurementNoiseCov, Scalar::all(1e-1));
    
    kf_initialized = true;
}

DetectedTarget GreenLightDetector::applyKalmanSmoothing(const DetectedTarget& raw_target) {
    if (!config.enable_kalman_filter || !kf_initialized) {
        return raw_target;
    }
    
    DetectedTarget smoothed = raw_target;
    
    if (raw_target.valid) {
        // 更新中心点滤波器
        Mat prediction_center = kf_center.predict();
        kf_center_measurement = (Mat_<float>(2, 1) << raw_target.center.x, raw_target.center.y);
        Mat correction_center = kf_center.correct(kf_center_measurement);
        smoothed.center.x = correction_center.at<float>(0);
        smoothed.center.y = correction_center.at<float>(1);
        
        // 更新半径滤波器
        Mat prediction_radius = kf_radius.predict();
        kf_radius_measurement = (Mat_<float>(1, 1) << raw_target.radius);
        Mat correction_radius = kf_radius.correct(kf_radius_measurement);
        smoothed.radius = correction_radius.at<float>(0);
    }
    
    return smoothed;
}

DetectedTarget GreenLightDetector::applyTemporalSmoothing(const DetectedTarget& raw_target) {
    DetectedTarget result = raw_target;
    
    // 添加到历史缓存
    frame_history.push_back(raw_target);
    if (frame_history.size() > static_cast<size_t>(config.smoothing_frames)) {
        frame_history.pop_front();
    }
    
    if (raw_target.valid) {
        valid_frames_count++;
        last_valid_target = raw_target;
    } else {
        valid_frames_count = 0;
    }
    
    // 如果当前帧无效，但前面有有效的历史帧，使用平均值
    if (!raw_target.valid && valid_frames_count < config.min_consecutive_frames) {
        int valid_count = 0;
        Point2f avg_center(0, 0);
        float avg_radius = 0;
        
        for (const auto& target : frame_history) {
            if (target.valid) {
                valid_count++;
                avg_center += target.center;
                avg_radius += target.radius;
            }
        }
        
        if (valid_count > 0) {
            result.center = avg_center / valid_count;
            result.radius = avg_radius / valid_count;
            result.valid = true;
        }
    }
    
    return result;
}

Mat GreenLightDetector::preprocessFrame(const Mat& frame) {
    Mat processed = frame.clone();
    
    // 应用高斯模糊以减少噪声
    if (config.gaussian_blur_sigma > 0) {
        int kernel_size = static_cast<int>(config.gaussian_blur_sigma * 2) * 2 + 1;
        GaussianBlur(processed, processed, Size(kernel_size, kernel_size), config.gaussian_blur_sigma);
    }
    
    // 应用双边滤波以保留边界同时平滑区域
    if (config.bilateral_filter_sigma > 0) {
        bilateralFilter(processed, processed, 9, config.bilateral_filter_sigma, config.bilateral_filter_sigma);
    }
    
    return processed;
}

float GreenLightDetector::calculateSimpleDistance(float radius_pixels) {
    // 使用相似三角形原理计算距离
    // distance = (真实直径 * 焦距) / (像素直径)
    if (radius_pixels <= 0) return 0;
    
    float diameter_pixels = radius_pixels * 2;
    float distance_mm = (config.real_diameter_mm * config.focal_length_pixels) / diameter_pixels;
    float distance_m = distance_mm / 1000.0f;
    
    return distance_m;
}

void GreenLightDetector::calculatePoseWithPNP(DetectedTarget& target) {
    if (!target.valid) return;
    
    // 更准确的PNP建模
    // 使用目标圆的边缘点建立3D-2D对应关系
    
    // 目标实际半径（从配置读取）
    float real_radius_m = config.real_diameter_mm / 2000.0f;  // 毫米转米
    
    // 创建更密集的点集以获得更好的精度
    vector<Point3f> object_points;
    int num_points = 6;  // 使用6个点，减少计算量
    
    for (int i = 0; i < num_points; i++) {
        float angle = 2 * CV_PI * i / num_points;
        float x = real_radius_m * cos(angle);
        float y = real_radius_m * sin(angle);
        float z = 0;
        object_points.push_back(Point3f(x, y, z));
    }
    
    // 添加圆心作为额外点（Z=0平面）
    object_points.push_back(Point3f(0, 0, 0));
    
    // 图像点
    vector<Point2f> image_points;
    float radius_px = target.radius;
    
    for (int i = 0; i < num_points; i++) {
        float angle = 2 * CV_PI * i / num_points;
        float x = target.center.x + radius_px * cos(angle);
        float y = target.center.y + radius_px * sin(angle);
        image_points.push_back(Point2f(x, y));
    }
    
    // 添加圆心对应的图像点
    image_points.push_back(Point2f(target.center.x, target.center.y));
    
    // 使用SolvePnP计算相机姿态
    Mat rvec, tvec;
    Mat camera_matrix = config.camera_matrix;
    Mat dist_coeffs = config.dist_coeffs;
    
    try {
        // 添加调试输出
        cout << "=== PNP计算调试信息 ===" << endl;
        cout << "目标半径(像素): " << radius_px << endl;
        cout << "目标实际半径(米): " << real_radius_m << endl;
        cout << "图像点数: " << image_points.size() << endl;
        
        bool success = solvePnP(object_points, image_points, 
                               camera_matrix, dist_coeffs, 
                               rvec, tvec, false, SOLVEPNP_ITERATIVE);
        
        if (success) {
            target.distance_pnp = static_cast<float>(tvec.at<double>(2, 0));
            
            target.world_position = Point3f(
                static_cast<float>(tvec.at<double>(0, 0)),
                static_cast<float>(tvec.at<double>(1, 0)),
                static_cast<float>(tvec.at<double>(2, 0))
            );
            
            // 计算重投影误差
            vector<Point2f> projected_points;
            projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);
            
            double total_error = 0;
            for (size_t i = 0; i < image_points.size(); i++) {
                double error = norm(image_points[i] - projected_points[i]);
                total_error += error * error;
            }
            // double mean_error = sqrt(total_error / image_points.size());  // 未使用
            
            cout << "PNP计算成功!" << endl;
            cout << "距离: " << target.distance_pnp << " 米" << endl;
            cout << "3D位置: (" << target.world_position.x << ", " 
                 << target.world_position.y << ", " << target.world_position.z << ")" << endl;
            cout << "==========================" << endl;
            
            // 计算姿态角
            calculateAngles(target);
        } else {
            cerr << "PNP计算失败" << endl;
        }
    }
    catch (const exception& e) {
        cerr << "PNP calculation error: " << e.what() << endl;
    }
}

void GreenLightDetector::calculateAngles(DetectedTarget& target) {
    if (!target.valid) return;
    
    // 计算在相机坐标系中的角度
    float x = target.world_position.x;
    float y = target.world_position.y;
     float z = target.world_position.z;
    
    // 计算距离
    float distance = sqrt(x*x + y*y + z*z);
    
    if (distance > 0) {
        // 俯仰角：绕X轴旋转，Y-Z平面
        target.pitch_deg = asin(y / distance) * RAD_TO_DEG;
        
        // 方位角：绕Y轴旋转，X-Z平面  
        target.yaw_deg = atan2(x, z) * RAD_TO_DEG;
    }
    
    // 转换到云台坐标系
    if (!config.R_camera2gimbal.empty() && !config.t_camera2gimbal.empty()) {
        // 将目标点在相机坐标系中的坐标转换为齐次坐标
        Mat point_camera = (Mat_<double>(4, 1) << 
            target.world_position.x, 
            target.world_position.y, 
            target.world_position.z, 
            1.0);
        
        // 构建相机到云台的变换矩阵
        Mat R = config.R_camera2gimbal;
        Mat t = config.t_camera2gimbal;
        
        Mat T_camera2gimbal = Mat::eye(4, 4, CV_64F);
        R.copyTo(T_camera2gimbal(Rect(0, 0, 3, 3)));
        t.copyTo(T_camera2gimbal(Rect(3, 0, 1, 3)));
        
        // 变换到云台坐标系
        Mat point_gimbal = T_camera2gimbal * point_camera;
        
        float x_g = static_cast<float>(point_gimbal.at<double>(0, 0));
        float y_g = static_cast<float>(point_gimbal.at<double>(1, 0));
        float z_g = static_cast<float>(point_gimbal.at<double>(2, 0));
        
        // 计算云台坐标系中的距离
        float distance_g = sqrt(x_g*x_g + y_g*y_g + z_g*z_g);
        
        if (distance_g > 0) {
            // 计算云台坐标系中的角度
            target.pitch_gimbal_deg = asin(y_g / distance_g) * RAD_TO_DEG;
            target.yaw_gimbal_deg = atan2(x_g, z_g) * RAD_TO_DEG;
        }
    }
}
bool GreenLightDetector::initCamera() {
    cout << "Initializing camera..." << endl;

    try {
        camera_ = std::make_unique<io::Camera>(config_path);
        cout << "Camera initialized successfully" << endl;
        return true;
    } catch (const std::exception& e) {
        cerr << "Failed to initialize camera: " << e.what() << endl;
        return false;
    }
}

// 更新processFrame函数，添加PNP计算
DetectedTarget GreenLightDetector::processFrame(cv::Mat& frame) {
    DetectedTarget result;
    result.valid = false;
    result.area = 0;
    
    // 转换为HSV颜色空间
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    
    // 创建绿色阈值掩码
    Scalar lower_green(config.h_min, config.s_min, config.v_min);
    Scalar upper_green(config.h_max, config.s_max, config.v_max);
    
    Mat mask;
    inRange(hsv, lower_green, upper_green, mask);
    
    // 形态学操作
    if (config.morph_open_size > 0) {
        Mat kernel_open = getStructuringElement(MORPH_ELLIPSE, 
            Size(config.morph_open_size, config.morph_open_size));
        morphologyEx(mask, mask, MORPH_OPEN, kernel_open);
    }
    
    if (config.morph_close_size > 0) {
        Mat kernel_close = getStructuringElement(MORPH_ELLIPSE, 
            Size(config.morph_close_size, config.morph_close_size));
        morphologyEx(mask, mask, MORPH_CLOSE, kernel_close);
    }
    
    // 查找轮廓
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    if (contours.empty()) {
        return result;
    }
    
    // 筛选最佳轮廓
    vector<Point> best_contour;
    float best_circularity = 0;
    float best_area = 0;
    
    for (const auto& contour : contours) {
        float area = static_cast<float>(contourArea(contour));
        if (area < config.min_area || area > config.max_area) {
            continue;
        }
        
        // 计算圆形度
        float perimeter = static_cast<float>(arcLength(contour, true));
        if (perimeter == 0) continue;
        
        float circularity = (4 * CV_PI * area) / (perimeter * perimeter);
        
        if (circularity > best_circularity && 
            circularity > config.min_circularity) {
            best_circularity = circularity;
            best_contour = contour;
            best_area = area;
        }
    }
    
    if (best_contour.empty()) {
        return result;
    }
    
    // 拟合最小外接圆
    Point2f center;
    float radius;
    minEnclosingCircle(best_contour, center, radius);
    
    // 计算边界框
    Rect bbox = boundingRect(best_contour);
    
    // 计算简单距离
    float distance_simple = calculateSimpleDistance(radius);
    
    result.center = center;
    result.boundingRect = bbox;
    result.radius = radius;
    result.distance_simple = distance_simple;
    result.circularity = best_circularity;
    result.area = best_area;
    result.valid = true;
    
    // 使用PNP计算更精确的距离和姿态
    calculatePoseWithPNP(result);
    
    return result;
}

// float GreenLightDetector::calculateDistance(float radius_pixels) {
//     // 使用相似三角形原理计算距离
//     // distance = (真实直径 * 焦距) / (像素直径)
//     if (radius_pixels <= 0) return 0;
    
//     float diameter_pixels = radius_pixels * 2;
//     float distance_mm = (config.real_diameter_mm * config.focal_length_pixels) / diameter_pixels;
//     float distance_m = distance_mm / 1000.0f;
    
//     return distance_m;
// }

// 更新sendDataToSerial函数，使用 SerialBoard 发送目标数据和电机转圈数
void GreenLightDetector::sendDataToSerial(const DetectedTarget& target) {
    // 检查是否成功初始化了串口
    if (!serial_board_) {
        return;
    }
    
    if (!target.valid) {
        return;
    }
    
    // 使用 SerialBoard 的 Command 结构发送数据
    // 目标的方位角(yaw)和俯仰角(pitch)映射到云台坐标系
    io::Command cmd;
    cmd.yaw = target.yaw_gimbal_deg;      // 使用云台坐标系的方位角
    cmd.pitch = target.pitch_gimbal_deg;  // 使用云台坐标系的俯仰角
    cmd.control = target.valid ? 1 : 0;   // 目标有效时为 1
    cmd.shoot = 0;                        // 不发射
    cmd.horizon_distance = target.distance_pnp > 0 ? 
                          target.distance_pnp : target.distance_simple;
    
    // 计算电机转圈数：根据云台方位角(yaw)与发射装置的水平距离
    // yaw 为目标相对于云台的方位角（度数），转换为对应的电机转圈数
    // 公式：电机转圈数 = yaw角度(度) / 360 * (motor_distance_per_rotation_mm / distance_per_rotation_mm)
    // 如果配置中启用了强制发送圈数，或处于固定策略
    if (config.force_motor_rotations || config.dart_strategy == Config::DartStrategy::Fixed) {
        cmd.motor_rotations = config.default_motor_rotations;
        tools::logger()->info("[Motor] force default motor_rotations={:.3f}", cmd.motor_rotations);
    } else if (config.dart_strategy == Config::DartStrategy::RandomFixed && detection_window_active) {
        // 在随机固定策略的窗口期间，保持当前位置
        cmd.motor_rotations = current_motor_rotations;
    } else if (config.motor_distance_per_rotation_mm > 0) {
        // 简化公式：目标方位角 / 360 表示相对于一整圈的比例
        // 电机转圈数 = 目标方位角比例 * 电机转圈数变化范围
        float motor_rotations = (target.yaw_gimbal_deg / 360.0f) * config.motor_distance_per_rotation_mm;
        
        // 限制电机转圈数在允许范围内
        motor_rotations = std::clamp(motor_rotations, config.motor_min_rotations, config.motor_max_rotations);
        
        cmd.motor_rotations = motor_rotations;
        
        tools::logger()->debug("[Motor] yaw_gimbal_deg={:.2f}°, motor_rotations={:.3f}", 
                              target.yaw_gimbal_deg, motor_rotations);
    } else {
        cmd.motor_rotations = 0.0;
    }
    
    // 发送命令到串口
    serial_board_->send(cmd);
    
    // 调试输出
    cout << "Sent to serial: yaw=" << cmd.yaw << "°, pitch=" << cmd.pitch 
         << "°, distance=" << cmd.horizon_distance << "m"
         << ", motor_rotations=" << cmd.motor_rotations << endl;
}

// 以下是 dart 事件的简单实现
void GreenLightDetector::onGateOpening() {
    if (config.dart_strategy != Config::DartStrategy::RandomFixed) return;
    detection_window_active = true;
    // 生成一个随机偏移
    double max_offset = (config.random_move_range_degrees / 360.0) * config.motor_distance_per_rotation_mm;
    std::uniform_real_distribution<double> dist(-max_offset, max_offset);
    current_motor_rotations = initial_motor_rotations + dist(rng);
    tools::logger()->info("[Dart] gate opening: move to {:.3f} rotations", current_motor_rotations);
}

void GreenLightDetector::onGateFullyOpened() {
    // 保留当前随机位置，不作额外处理
    if (config.dart_strategy != Config::DartStrategy::RandomFixed) return;
}

void GreenLightDetector::onDetectionWindowEnd() {
    if (config.dart_strategy != Config::DartStrategy::RandomFixed) return;
    detection_window_active = false;
    current_motor_rotations = initial_motor_rotations;
    tools::logger()->info("[Dart] window end, restore initial position");
}

void GreenLightDetector::onDartHit() {
    if (config.dart_strategy != Config::DartStrategy::RandomFixed) return;
    if (config.random_move_after_hit) {
        double max_offset = (config.random_move_range_degrees / 360.0) * config.motor_distance_per_rotation_mm;
        std::uniform_real_distribution<double> dist(-max_offset, max_offset);
        current_motor_rotations = initial_motor_rotations + dist(rng);
        tools::logger()->info("[Dart] hit -> new random position {:.3f}", current_motor_rotations);
    }
}


// 更新drawResults函数，显示PNP计算结果
void GreenLightDetector::drawResults(cv::Mat& frame, const DetectedTarget& target) {
    // 显示FPS
    static int frame_count = 0;
    static auto last_time = chrono::steady_clock::now();
    static float fps = 0;
    
    frame_count++;
    auto current_time = chrono::steady_clock::now();
    auto elapsed = chrono::duration_cast<chrono::milliseconds>(current_time - last_time).count();
    
    if (elapsed > 1000) {
        fps = frame_count * 1000.0f / elapsed;
        frame_count = 0;
        last_time = current_time;
    }
    
    // 绘制FPS
    string fps_text = format("FPS: %.1f", fps);
    putText(frame, fps_text, Point(frame.cols - 150, 30), 
            FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
    
    if (!target.valid) {
        // 如果没有检测到目标
        string no_target_text = "NO TARGET";
        int baseline = 0;
        Size text_size = getTextSize(no_target_text, FONT_HERSHEY_SIMPLEX, 2, 3, &baseline);
        Point text_org((frame.cols - text_size.width) / 2, (frame.rows + text_size.height) / 2);
        putText(frame, no_target_text, text_org, FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 3);
        return;
    }
    
    // 绘制检测结果
    if (config.draw_bounding_box) {
        rectangle(frame, target.boundingRect, Scalar(0, 255, 0), 2);
    }
    
    if (config.draw_center) {
        circle(frame, target.center, 5, Scalar(0, 0, 255), -1);
        line(frame, Point(static_cast<int>(target.center.x - 10), static_cast<int>(target.center.y)),
             Point(static_cast<int>(target.center.x + 10), static_cast<int>(target.center.y)), Scalar(0, 0, 255), 2);
        line(frame, Point(static_cast<int>(target.center.x), static_cast<int>(target.center.y - 10)),
             Point(static_cast<int>(target.center.x), static_cast<int>(target.center.y + 10)), Scalar(0, 0, 255), 2);
    }
    
    if (config.show_distance) {
        // 绘制信息面板
        int y_offset = 30;
        int line_height = 25;
        
        // 选择距离显示（优先显示PNP距离）
        float display_distance = target.distance_pnp > 0 ? target.distance_pnp : target.distance_simple;
        string method = target.distance_pnp > 0 ? "(PNP)" : "(Simple)";
        
        string distance_text = format("Distance %s: %.2f m", method.c_str(), display_distance);
        putText(frame, distance_text, Point(10, y_offset), 
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        
        string position_text = format("Image Pos: (%.0f, %.0f)", target.center.x, target.center.y);
        putText(frame, position_text, Point(10, y_offset + line_height), 
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        
        if (target.distance_pnp > 0 && config.show_pnp_results) {
            // 显示3D位置
            string world_pos_text = format("3D Pos: (%.2f, %.2f, %.2f) m", 
                target.world_position.x, target.world_position.y, target.world_position.z);
            putText(frame, world_pos_text, Point(10, y_offset + line_height * 2), 
                    FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
            
            // 显示相机坐标系角度
            string angles_text = format("Camera Angles: Pitch=%.1f°, Yaw=%.1f°", 
                target.pitch_deg, target.yaw_deg);
            putText(frame, angles_text, Point(10, y_offset + line_height * 3), 
                    FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
            
            // 显示云台坐标系角度
            if (target.pitch_gimbal_deg != 0 || target.yaw_gimbal_deg != 0) {
                string gimbal_angles_text = format("Gimbal Angles: Pitch=%.1f°, Yaw=%.1f°", 
                    target.pitch_gimbal_deg, target.yaw_gimbal_deg);
                putText(frame, gimbal_angles_text, Point(10, y_offset + line_height * 4), 
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 200, 0), 2);
            }
        }
        
        // 显示圆形度和面积
        string circle_text = format("Circularity: %.2f", target.circularity);
        putText(frame, circle_text, Point(frame.cols - 200, y_offset + line_height), 
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(200, 200, 200), 2);
        
        string area_text = format("Area: %.0f px^2", target.area);
        putText(frame, area_text, Point(frame.cols - 200, y_offset + line_height * 2), 
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(200, 200, 200), 2);
    }
}


void GreenLightDetector::run() {
    // 初始化相机
    if (!initCamera()) {
        return;
    }

    Mat frame;
    
    cout << "Starting detection loop. Press 'q' to quit." << endl;
    cout << "Press 'p' to toggle PNP results display." << endl;
    
    while (true) {
        std::chrono::steady_clock::time_point timestamp;
        camera_->read(frame, timestamp);
        
        if (frame.empty()) {
            cerr << "Failed to read frame" << endl;
            this_thread::sleep_for(chrono::milliseconds(100));
            continue;
        }
        
        // 处理帧
        DetectedTarget target = processFrame(frame);
        
        // 发送数据到串口
        sendDataToSerial(target);
        
        // 绘制结果
        drawResults(frame, target);
        
        // 显示结果
        if (config.show_preview) {
            Mat display_frame;
            if (config.window_scale != 1.0) {
                resize(frame, display_frame, Size(), 
                       config.window_scale, config.window_scale);
            } else {
                display_frame = frame;
            }
            
             // 转换为RGB颜色空间
            Mat rgb_display;
            cvtColor(display_frame, rgb_display, COLOR_BGR2RGB);
            
            imshow("Green Light Detection", rgb_display);
        }
        
        // 按键处理
        int key = waitKey(1);
        if (key == 'q' || key == 27) { // 'q' or ESC
            cout << "Exiting..." << endl;
            break;
        } else if (key == 'p') {
            config.show_pnp_results = !config.show_pnp_results;
            cout << "PNP results display: " << (config.show_pnp_results ? "ON" : "OFF") << endl;
        } else if (key == 'g') {
            // 模拟闸门开始打开
            cout << "[TEST] gate opening event" << endl;
            onGateOpening();
        } else if (key == 'e') {
            // 模拟检测窗口结束
            cout << "[TEST] window end event" << endl;
            onDetectionWindowEnd();
        } else if (key == 'h') {
            // 模拟飞镖命中
            cout << "[TEST] dart hit event" << endl;
            onDartHit();
        }
    }
}

