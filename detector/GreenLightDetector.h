#ifndef GREEN_LIGHT_DETECTOR_H
#define GREEN_LIGHT_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

// 前向声明，避免循环依赖
class SerialPort;
struct ReceivedFrame;  // 前向声明
class ProtocolHandler; // 前向声明

#include "../serial/serial.hpp"
#include "../serial/protocol_handler.hpp"  // 包含协议处理器头文件
#include "../io/camera.hpp"

// 检测到的目标结构体
struct DetectedTarget {
    cv::Point2f center;         // 图像中心坐标（像素）
    cv::Rect boundingRect;      // 边界框
    float radius;              // 半径（像素）
    float distance_simple;     // 相似三角形计算的距离（米）
    float distance_pnp;        // PNP计算的距离（米）
    float circularity;         // 圆形度
    float area;                // 面积（像素）
    float pitch_deg;           // 俯仰角（度）相机坐标系
    float yaw_deg;             // 方位角（度）相机坐标系
    float pitch_gimbal_deg;    // 俯仰角（度）云台坐标系
    float yaw_gimbal_deg;      // 方位角（度）云台坐标系
    cv::Point3f world_position; // 3D世界坐标（米）
    bool valid;               // 是否有效
    
    // 默认构造函数
    DetectedTarget() : center(0, 0), boundingRect(), radius(0), 
                      distance_simple(0), distance_pnp(0), circularity(0), area(0),
                      pitch_deg(0), yaw_deg(0), pitch_gimbal_deg(0), yaw_gimbal_deg(0),
                      world_position(0, 0, 0), valid(false) {}
};

// 配置结构体
struct Config {
    // 相机配置
    std::string camera_name;
    float exposure_ms;
    float gain;
    int width;
    int height;
    int fps;
    std::string vid_pid;
    
    // 图像处理配置
    int h_min, h_max;
    int s_min, s_max;
    int v_min, v_max;
    int morph_open_size;
    int morph_close_size;
    float min_circularity;
    int min_area;
    int max_area;
    
    // 距离计算配置
    float real_diameter_mm;          // 实际直径（毫米）
    float focal_length_pixels;       // 焦距（像素）
    
    // 相机内参（从配置文件中读取）
    cv::Mat camera_matrix;           // 3x3 相机内参矩阵
    cv::Mat dist_coeffs;             // 畸变系数
    
    // 相机-云台转换
    cv::Mat R_camera2gimbal;         // 3x3 旋转矩阵
    cv::Mat t_camera2gimbal;         // 3x1 平移向量
    
    // 串口配置
    std::string serial_port;
    int serial_baudrate;
    std::string serial_data_format;
    
    // 显示配置
    bool show_preview;
    bool draw_bounding_box;
    bool draw_center;
    bool show_distance;
    bool show_mask;
    bool show_pnp_results;           // 是否显示PNP计算结果
    float window_scale;
    
    // PNP计算相关配置
    float target_radius_3d;          // 目标在3D空间中的半径（米）
};

// 主检测器类
class GreenLightDetector {
private:
    Config config;
    std::unique_ptr<SerialPort> serial_port;
    std::unique_ptr<ProtocolHandler> protocol_handler;
    std::unique_ptr<io::Camera> camera_;
    std::string config_path;
    
    // 当前检测目标
    DetectedTarget target;
    
    // 从文件加载配置（包括相机内参和相机-云台转换）
    bool loadConfig(const std::string& config_path);
    
    // 加载相机矩阵和转换矩阵
    bool loadCameraParameters();
    
    // 初始化相机
    bool initCamera();
    
    // 处理单帧图像
    DetectedTarget processFrame(cv::Mat& frame);
    
    // 计算目标距离（相似三角形方法）
    float calculateSimpleDistance(float radius_pixels);
    
    // 使用PNP计算3D位置和距离
    void calculatePoseWithPNP(DetectedTarget& target);
    
    // 计算姿态角（俯仰角和方位角）
    void calculateAngles(DetectedTarget& target);
    
    // 发送数据到串口
    void sendDataToSerial(const DetectedTarget& target);
    
    // 在图像上绘制结果
    void drawResults(cv::Mat& frame, const DetectedTarget& target);
    
    // 新增：串口数据处理
    void processSerialData();
    void onFrameReceived(const ReceivedFrame& frame);
    void generateResponse(const ReceivedFrame& frame);
    
public:
    GreenLightDetector();
    ~GreenLightDetector();
    
    // 初始化系统
    bool initialize(const std::string& config_path = "config.yaml");
    
    // 主运行循环
    void run();
    
    // 获取当前配置
    Config getConfig() const { return config; }
    
    // 设置配置
    void setConfig(const Config& new_config);
};

#endif // GREEN_LIGHT_DETECTOR_H