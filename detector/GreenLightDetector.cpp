#include "GreenLightDetector.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>
#include <memory>
#include <algorithm>
#include <iomanip>

using namespace std;
using namespace cv;

// GreenLightDetector 实现
GreenLightDetector::GreenLightDetector() : camera_initialized(false) {
    serial_port = std::unique_ptr<SerialPort>(new SerialPort());
}

GreenLightDetector::~GreenLightDetector() {
    if (cap.isOpened()) {
        cap.release();
    }
}

bool GreenLightDetector::loadConfig(const std::string& config_path) {
    try {
        YAML::Node config_yaml = YAML::LoadFile(config_path);
        
        // Camera Configuration
        if (config_yaml["camera_name"]) config.camera_name = config_yaml["camera_name"].as<std::string>();
        if (config_yaml["exposure_ms"]) config.exposure_ms = config_yaml["exposure_ms"].as<float>();
        if (config_yaml["gain"]) config.gain = config_yaml["gain"].as<float>();
        if (config_yaml["width"]) config.width = config_yaml["width"].as<int>();
        if (config_yaml["height"]) config.height = config_yaml["height"].as<int>();
        if (config_yaml["fps"]) config.fps = config_yaml["fps"].as<int>();
        if (config_yaml["vid_pid"]) config.vid_pid = config_yaml["vid_pid"].as<std::string>();
        
        // Image Processing Configuration
        if (config_yaml["h_min"]) config.h_min = config_yaml["h_min"].as<int>();
        if (config_yaml["h_max"]) config.h_max = config_yaml["h_max"].as<int>();
        if (config_yaml["s_min"]) config.s_min = config_yaml["s_min"].as<int>();
        if (config_yaml["s_max"]) config.s_max = config_yaml["s_max"].as<int>();
        if (config_yaml["v_min"]) config.v_min = config_yaml["v_min"].as<int>();
        if (config_yaml["v_max"]) config.v_max = config_yaml["v_max"].as<int>();
        if (config_yaml["morph_open_size"]) config.morph_open_size = config_yaml["morph_open_size"].as<int>();
        if (config_yaml["morph_close_size"]) config.morph_close_size = config_yaml["morph_close_size"].as<int>();
        if (config_yaml["min_circularity"]) config.min_circularity = config_yaml["min_circularity"].as<float>();
        if (config_yaml["min_area"]) config.min_area = config_yaml["min_area"].as<int>();
        if (config_yaml["max_area"]) config.max_area = config_yaml["max_area"].as<int>();
        
        // Distance Calculation
        if (config_yaml["real_diameter_mm"]) config.real_diameter_mm = config_yaml["real_diameter_mm"].as<float>();
        if (config_yaml["focal_length_pixels"]) config.focal_length_pixels = config_yaml["focal_length_pixels"].as<float>();
        
        // Serial Port Configuration
        if (config_yaml["serial_port"]) config.serial_port = config_yaml["serial_port"].as<std::string>();
        if (config_yaml["serial_baudrate"]) config.serial_baudrate = config_yaml["serial_baudrate"].as<int>();
        if (config_yaml["serial_data_format"]) config.serial_data_format = config_yaml["serial_data_format"].as<std::string>();
        
        // Display Configuration
        if (config_yaml["show_preview"]) config.show_preview = config_yaml["show_preview"].as<bool>();
        if (config_yaml["draw_bounding_box"]) config.draw_bounding_box = config_yaml["draw_bounding_box"].as<bool>();
        if (config_yaml["draw_center"]) config.draw_center = config_yaml["draw_center"].as<bool>();
        if (config_yaml["show_distance"]) config.show_distance = config_yaml["show_distance"].as<bool>();
        if (config_yaml["show_mask"]) config.show_mask = config_yaml["show_mask"].as<bool>();
        if (config_yaml["window_scale"]) config.window_scale = config_yaml["window_scale"].as<float>();
        
        cout << "Configuration loaded from " << config_path << endl;
        return true;
    }
    catch (const YAML::Exception& e) {
        cerr << "Error loading config file: " << e.what() << endl;
        return false;
    }
}

bool GreenLightDetector::initialize(const std::string& config_path) {
    this->config_path = config_path;
    
    // 初始化串口
    if (!serial_port->open(config.serial_port, config.serial_baudrate)) {
        cerr << "Warning: Serial port initialization failed. Continuing without serial..." << endl;
        cerr << "To fix: sudo chmod 666 " << config.serial_port << endl;
    } else {
        cout << "Serial port initialized successfully" << endl;
    }
    
    return true;
}

bool GreenLightDetector::initCamera() {
    cout << "Initializing camera..." << endl;
    
    // 首先尝试通过V4L2打开
    cout << "Trying V4L2 device /dev/video1..." << endl;
    cap.open("/dev/video1", CAP_V4L2);
    
    // 如果失败，尝试其他设备
    if (!cap.isOpened()) {
        for (int i = 0; i < 10; i++) {
            cout << "Trying camera index " << i << "..." << endl;
            cap.open(i);
            if (cap.isOpened()) break;
        }
    }
    
    // 如果还是失败，尝试视频文件（用于测试）
    if (!cap.isOpened()) {
        cout << "Trying test video file..." << endl;
        cap.open("test_video.mp4");
        if (cap.isOpened()) {
            cout << "Test video loaded. Running in test mode." << endl;
            camera_initialized = true;
            return true;
        }
    }
    
    if (!cap.isOpened()) {
        cerr << "ERROR: Failed to open camera or video source" << endl;
        cerr << "Please check:" << endl;
        cerr << "1. Camera is connected and powered on" << endl;
        cerr << "2. Permissions: ls -l /dev/video*" << endl;
        cerr << "3. Try: sudo usermod -a -G video $USER (then logout/login)" << endl;
        cerr << "4. Create test video: ffmpeg -f lavfi -i testsrc=size=1280x720:rate=30:duration=60 test_video.mp4" << endl;
        return false;
    }
    
    camera_initialized = true;
    return true;
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
    
    char buffer[256];
    snprintf(buffer, sizeof(buffer), config.serial_data_format.c_str(),
             target.center.x, target.center.y, target.distance);
    
    string data(buffer);
    int bytes_written = serial_port->write(data);
    
    if (bytes_written > 0) {
        // 在终端显示发送的数据
        cout << "Serial: " << data;
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
    // 加载配置
    if (!loadConfig(config_path)) {
        cerr << "Error: Failed to load configuration from " << config_path << endl;
        return;
    }
    
    // 初始化相机
    if (!initCamera()) {
        return;
    }
    
    Mat frame;
    
    cout << "Starting detection loop. Press 'q' to quit." << endl;
    
    while (true) {
        if (!cap.read(frame) || frame.empty()) {
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

