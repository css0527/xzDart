#ifndef GREEN_LIGHT_DETECTOR_H
#define GREEN_LIGHT_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>
#include "../serial/serial.hpp"

// 检测到的目标结构体
struct DetectedTarget {
    cv::Point2f center;
    cv::Rect boundingRect;
    float radius;
    float distance;
    float circularity;
    float area;
    bool valid;
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
    float real_diameter_mm;
    float focal_length_pixels;
    
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
    float window_scale;
};

// 主检测器类
class GreenLightDetector {
private:
    Config config;
    std::unique_ptr<SerialPort> serial_port;
    cv::VideoCapture cap;
    bool camera_initialized;
    std::string config_path;
    
    // 从文件加载配置
    bool loadConfig(const std::string& config_path);
    
    // 初始化相机
    bool initCamera();
    
    // 处理单帧图像
    DetectedTarget processFrame(cv::Mat& frame);
    
    // 计算目标距离
    float calculateDistance(float radius_pixels);
    
    // 发送数据到串口
    void sendDataToSerial(const DetectedTarget& target);
    
    // 在图像上绘制结果
    void drawResults(cv::Mat& frame, const DetectedTarget& target);
    
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