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

// 确保先包含协议处理器头文件
#include "../serial/protocol_handler.hpp"
#include "../tools/yaml.hpp"

using namespace std;
using namespace cv;

// GreenLightDetector 实现
GreenLightDetector::GreenLightDetector() 
    : serial_port(std::unique_ptr<SerialPort>(new SerialPort()))
    , protocol_handler(std::unique_ptr<ProtocolHandler>(new ProtocolHandler())) {
}

GreenLightDetector::~GreenLightDetector() {
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
    
    // 初始化串口
    if (!serial_port->open(config.serial_port, config.serial_baudrate)) {
        cerr << "Warning: Serial port initialization failed. Continuing without serial..." << endl;
        cerr << "To fix: sudo chmod 666 " << config.serial_port << endl;
    } else {
        cout << "Serial port initialized successfully" << endl;

         // 配置协议处理器
        // 设置接收帧头帧尾（根据实际协议）
        std::vector<uint8_t> rx_header = {0xAA, 0x55};  // 接收帧头
        std::vector<uint8_t> rx_footer = {0x55, 0xAA};  // 接收帧尾
        protocol_handler->setReceiveHeader(rx_header);
        protocol_handler->setReceiveFooter(rx_footer);
        
        // 设置发送帧头帧尾
        std::vector<uint8_t> tx_header = {0xBB, 0x66};  // 发送帧头
        std::vector<uint8_t> tx_footer = {0x66, 0xBB};  // 发送帧尾
        protocol_handler->setTransmitHeader(tx_header);
        protocol_handler->setTransmitFooter(tx_footer);
        
        // 设置CRC校验
        protocol_handler->setUseCRC(true);
        
        // 设置回调函数
        protocol_handler->setFrameReceivedCallback(
            [this](const ReceivedFrame& frame) {
                this->onFrameReceived(frame);
            }
        );
        
        // 启动异步处理
        protocol_handler->startAsyncProcessing();
    }
    
    return true;
}
// 新增：串口数据处理线程
void GreenLightDetector::processSerialData() {
    const int BUFFER_SIZE = 1024;
    uint8_t buffer[BUFFER_SIZE];
    
    while (serial_port->isOpen()) {
        int bytes_read = serial_port->read(buffer, BUFFER_SIZE, 10); // 10ms超时
        
        if (bytes_read > 0) {
            // 将数据喂给协议处理器
            protocol_handler->feedData(buffer, bytes_read);
            
            // 也可以直接处理原始数据
            // cout << "Received raw data: ";
            // for (int i = 0; i < bytes_read; ++i) {
            //     printf("%02X ", buffer[i]);
            // }
            // cout << endl;
        }
        
        // 短暂休眠，避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// 新增：接收到完整帧的回调
void GreenLightDetector::onFrameReceived(const ReceivedFrame& frame) {
    cout << "Received frame: " << endl;
    cout << "  Header: ";
    for (auto b : frame.header) printf("%02X ", b);
    cout << endl;
    
    cout << "  Data: ";
    for (auto b : frame.data) {
        if (b >= 32 && b <= 126) {
            cout << static_cast<char>(b);
        } else {
            printf("[%02X]", b);
        }
    }
    cout << endl;
    
    cout << "  Footer: ";
    for (auto b : frame.footer) printf("%02X ", b);
    cout << endl;
    
    cout << "  CRC: " << (frame.crc_valid ? "Valid" : "Invalid") << endl;
    
    // 生成响应数据
    generateResponse(frame);
}

// 新增：根据接收数据生成响应
void GreenLightDetector::generateResponse(const ReceivedFrame& frame) {
    if (!serial_port->isOpen()) return;
    
    // 示例：解析接收到的命令
    if (!frame.data.empty()) {
        // 假设第一个字节是命令码
        uint8_t command = frame.data[0];
        
        std::vector<uint8_t> response_data;
        
        switch (command) {
            case 0x01:  // 获取状态
                response_data = {0x01, 0x00};  // 状态正常
                break;
                
            case 0x02:  // 获取检测结果
                // 这里可以填充实际的检测数据
                {
                    float x = target.center.x;
                    float y = target.center.y;
                    float d = target.distance;
                    
                    uint8_t* x_bytes = reinterpret_cast<uint8_t*>(&x);
                    uint8_t* y_bytes = reinterpret_cast<uint8_t*>(&y);
                    uint8_t* d_bytes = reinterpret_cast<uint8_t*>(&d);
                    
                    response_data = {0x02};  // 命令码
                    for (int i = 0; i < 4; ++i) response_data.push_back(x_bytes[i]);
                    for (int i = 0; i < 4; ++i) response_data.push_back(y_bytes[i]);
                    for (int i = 0; i < 4; ++i) response_data.push_back(d_bytes[i]);
                }
                break;
                
            case 0x03:  // 设置参数
                // 解析设置参数并更新配置
                // ... 参数解析代码
                response_data = {0x03, 0x01};  // 设置成功
                break;
                
            default:
                response_data = {0xFF, 0x00};  // 未知命令
                break;
        }
        
        // 打包数据并发送
        auto packet = protocol_handler->packData(response_data, true);
        serial_port->write(packet.data(), packet.size());
        
        cout << "Sent response packet" << endl;
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
    
    // 计算距离
    float distance = calculateDistance(radius);
    
    result.center = center;
    result.boundingRect = bbox;
    result.radius = radius;
    result.distance = distance;
    result.circularity = best_circularity;
    result.area = best_area;
    result.valid = true;
    
    return result;
}

float GreenLightDetector::calculateDistance(float radius_pixels) {
    // 使用相似三角形原理计算距离
    // distance = (真实直径 * 焦距) / (像素直径)
    if (radius_pixels <= 0) return 0;
    
    float diameter_pixels = radius_pixels * 2;
    float distance_mm = (config.real_diameter_mm * config.focal_length_pixels) / diameter_pixels;
    float distance_m = distance_mm / 1000.0f;
    
    return distance_m;
}

void GreenLightDetector::sendDataToSerial(const DetectedTarget& target) {
    if (!serial_port->isOpen() || !target.valid) {
        return;
    }
    
    
    // 准备数据
    std::vector<uint8_t> data;
    
    // 数据格式：数据类型(1字节) + X坐标(4字节) + Y坐标(4字节) + 距离(4字节)
    data.push_back(0xA0);  // 数据类型：检测结果
    
    // 添加X坐标（float转4字节）
    float x = target.center.x;
    uint8_t* x_bytes = reinterpret_cast<uint8_t*>(&x);
    for (int i = 0; i < 4; ++i) data.push_back(x_bytes[i]);
    
    // 添加Y坐标
    float y = target.center.y;
    uint8_t* y_bytes = reinterpret_cast<uint8_t*>(&y);
    for (int i = 0; i < 4; ++i) data.push_back(y_bytes[i]);
    
    // 添加距离
    float d = target.distance;
    uint8_t* d_bytes = reinterpret_cast<uint8_t*>(&d);
    for (int i = 0; i < 4; ++i) data.push_back(d_bytes[i]);
    
    // 使用协议处理器打包数据（包含帧头帧尾和CRC）
    auto packet = protocol_handler->packData(data, true);
    
    // 发送数据
    int bytes_written = serial_port->write(packet.data(), packet.size());
    
    if (bytes_written > 0) {
        // 显示调试信息
        cout << "Sent detection data: ";
        cout << "X=" << x << ", Y=" << y << ", D=" << d;
        cout << " (" << bytes_written << " bytes)" << endl;
    }
}

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
        int line_height = 30;
        
        string distance_text = format("Distance: %.2f m", target.distance);
        putText(frame, distance_text, Point(10, y_offset), 
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
        
        string position_text = format("Position: (%.0f, %.0f)", target.center.x, target.center.y);
        putText(frame, position_text, Point(10, y_offset + line_height), 
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
        
        string circularity_text = format("Circularity: %.2f", target.circularity);
        putText(frame, circularity_text, Point(10, y_offset + line_height * 2), 
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
        
        string radius_text = format("Radius: %.1f px", target.radius);
        putText(frame, radius_text, Point(10, y_offset + line_height * 3), 
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
        
        string area_text = format("Area: %.0f px^2", target.area);
        putText(frame, area_text, Point(10, y_offset + line_height * 4), 
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
    }
}

void GreenLightDetector::run() {
    // 初始化相机
    if (!initCamera()) {
        return;
    }
    
    // 启动串口数据读取线程（如果串口已打开）
    std::thread serial_thread;
    if (serial_port->isOpen()) {
        serial_thread = std::thread([this]() {
            this->processSerialData();
        });
    }

    Mat frame;
    
    cout << "Starting detection loop. Press 'q' to quit." << endl;
    
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
            
            imshow("Green Light Detection", display_frame);
        }
        
        // 按键处理
        int key = waitKey(1);
        if (key == 'q' || key == 27) { // 'q' or ESC
            cout << "Exiting..." << endl;
            break;
        }
    }
}

void GreenLightDetector::setConfig(const Config& new_config) {
    config = new_config;
}

