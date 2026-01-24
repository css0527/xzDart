#include "protocol_handler.hpp"
#include "crc.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>

using namespace std;
using namespace std::chrono;

ProtocolHandler::ProtocolHandler() 
    : use_crc(true)
    , async_mode(false)
    , max_buffer_size(4096)
    , running(false) {
    
    // 设置默认帧头帧尾
    rx_header = {0xAA, 0x55};
    rx_footer = {0x55, 0xAA};
    tx_header = {0xBB, 0x66};
    tx_footer = {0x66, 0xBB};
}

ProtocolHandler::~ProtocolHandler() {
    stopAsyncProcessing();
}

// CRC计算
uint16_t ProtocolHandler::calculateCRC(const uint8_t* data, size_t length) {
    return crc16(data, length);
}

// 设置接收帧头
void ProtocolHandler::setReceiveHeader(const vector<uint8_t>& header) {
    rx_header = header;
}

// 设置接收帧尾
void ProtocolHandler::setReceiveFooter(const vector<uint8_t>& footer) {
    rx_footer = footer;
}

// 设置发送帧头
void ProtocolHandler::setTransmitHeader(const vector<uint8_t>& header) {
    tx_header = header;
}

// 设置发送帧尾
void ProtocolHandler::setTransmitFooter(const vector<uint8_t>& footer) {
    tx_footer = footer;
}

// 设置回调函数
void ProtocolHandler::setFrameReceivedCallback(FrameReceivedCallback callback) {
    frame_callback = callback;
}

void ProtocolHandler::setRawDataCallback(RawDataCallback callback) {
    raw_callback = callback;
}

// 帧头匹配
bool ProtocolHandler::matchHeader(const vector<uint8_t>& data, size_t pos) {
    if (pos + rx_header.size() > data.size()) {
        return false;
    }
    
    for (size_t i = 0; i < rx_header.size(); ++i) {
        if (data[pos + i] != rx_header[i]) {
            return false;
        }
    }
    return true;
}

// 帧尾匹配
bool ProtocolHandler::matchFooter(const vector<uint8_t>& data, size_t pos) {
    if (pos + rx_footer.size() > data.size()) {
        return false;
    }
    
    for (size_t i = 0; i < rx_footer.size(); ++i) {
        if (data[pos + i] != rx_footer[i]) {
            return false;
        }
    }
    return true;
}

// 喂入数据
void ProtocolHandler::feedData(const uint8_t* data, size_t length) {
    if (async_mode) {
        // 异步模式：将数据放入队列
        lock_guard<mutex> lock(buffer_mutex);
        data_queue.push(vector<uint8_t>(data, data + length));
        data_cond.notify_one();
    } else {
        // 同步模式：直接处理
        processReceivedData(vector<uint8_t>(data, data + length));
    }
}

void ProtocolHandler::feedData(const vector<uint8_t>& data) {
    feedData(data.data(), data.size());
}

// 处理接收到的数据
void ProtocolHandler::processReceivedData(const vector<uint8_t>& data) {
    // 将新数据添加到缓冲区
    rx_buffer.insert(rx_buffer.end(), data.begin(), data.end());
    
    // 限制缓冲区大小
    if (rx_buffer.size() > max_buffer_size) {
        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + (rx_buffer.size() - max_buffer_size));
    }
    
    // 处理缓冲区中的数据
    size_t pos = 0;
    while (pos < rx_buffer.size()) {
        // 查找帧头
        if (!matchHeader(rx_buffer, pos)) {
            pos++;
            continue;
        }
        
        // 找到帧头，开始查找帧尾
        size_t frame_start = pos;
        size_t data_start = pos + rx_header.size();
        size_t footer_start = 0;
        
        // 查找帧尾
        for (size_t i = data_start; i <= rx_buffer.size() - rx_footer.size(); ++i) {
            if (matchFooter(rx_buffer, i)) {
                footer_start = i;
                break;
            }
        }
        
        if (footer_start == 0) {
            // 没有找到完整的帧，保留剩余数据
            if (frame_start > 0) {
                rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + frame_start);
            }
            break;
        }
        
        // 提取完整的帧
        size_t frame_end = footer_start + rx_footer.size();
        size_t data_length = footer_start - data_start;
        
        ReceivedFrame frame;
        frame.header = vector<uint8_t>(rx_buffer.begin() + frame_start, 
                                       rx_buffer.begin() + data_start);
        frame.data = vector<uint8_t>(rx_buffer.begin() + data_start,
                                     rx_buffer.begin() + footer_start);
        frame.footer = vector<uint8_t>(rx_buffer.begin() + footer_start,
                                       rx_buffer.begin() + frame_end);
        
        // CRC校验
        if (use_crc && data_length >= 2) {
            // 假设最后2个字节是CRC
            frame.crc = (frame.data[data_length - 2] << 8) | 
                        frame.data[data_length - 1];
            
            // 计算数据的CRC（排除CRC字节）
            uint16_t calculated_crc = calculateCRC(frame.data.data(), data_length - 2);
            frame.crc_valid = (frame.crc == calculated_crc);
            
            // 移除CRC字节
            if (frame.crc_valid && data_length >= 2) {
                frame.data.resize(data_length - 2);
            }
        } else {
            frame.crc = 0;
            frame.crc_valid = true;
        }
        
        // 调用回调函数
        if (frame_callback) {
            frame_callback(frame);
        }
        
        // 调用原始数据回调
        if (raw_callback) {
            vector<uint8_t> raw_frame(rx_buffer.begin() + frame_start, 
                                      rx_buffer.begin() + frame_end);
            raw_callback(raw_frame);
        }
        
        // 移动到下一帧
        pos = frame_end;
        
        // 清理已处理的数据
        if (pos > 0) {
            rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + pos);
            pos = 0;
        }
    }
}

// 数据打包
vector<uint8_t> ProtocolHandler::packData(const vector<uint8_t>& data, bool include_crc) {
    return packData(data.data(), data.size(), include_crc);
}

vector<uint8_t> ProtocolHandler::packData(const uint8_t* data, size_t length, bool include_crc) {
    vector<uint8_t> packet;
    
    // 添加发送帧头
    packet.insert(packet.end(), tx_header.begin(), tx_header.end());
    
    // 添加数据
    packet.insert(packet.end(), data, data + length);
    
    // 添加CRC（如果需要）
    if (include_crc && use_crc) {
        uint16_t crc = calculateCRC(data, length);
        packet.push_back(static_cast<uint8_t>(crc >> 8));
        packet.push_back(static_cast<uint8_t>(crc & 0xFF));
    }
    
    // 添加发送帧尾
    packet.insert(packet.end(), tx_footer.begin(), tx_footer.end());
    
    return packet;
}

// 数据解包（同步模式）
vector<ReceivedFrame> ProtocolHandler::unpackData(const vector<uint8_t>& data) {
    // 临时处理数据
    vector<ReceivedFrame> frames;
    
    // 创建临时副本进行处理
    vector<uint8_t> temp_buffer = data;
    
    // 查找并解析帧
    size_t pos = 0;
    while (pos < temp_buffer.size()) {
        // 查找帧头
        if (!matchHeader(temp_buffer, pos)) {
            pos++;
            continue;
        }
        
        size_t frame_start = pos;
        size_t data_start = pos + rx_header.size();
        size_t footer_start = 0;
        
        // 查找帧尾
        for (size_t i = data_start; i <= temp_buffer.size() - rx_footer.size(); ++i) {
            if (matchFooter(temp_buffer, i)) {
                footer_start = i;
                break;
            }
        }
        
        if (footer_start == 0) break;
        
        size_t frame_end = footer_start + rx_footer.size();
        size_t data_length = footer_start - data_start;
        
        ReceivedFrame frame;
        frame.header = vector<uint8_t>(temp_buffer.begin() + frame_start,
                                       temp_buffer.begin() + data_start);
        frame.data = vector<uint8_t>(temp_buffer.begin() + data_start,
                                     temp_buffer.begin() + footer_start);
        frame.footer = vector<uint8_t>(temp_buffer.begin() + footer_start,
                                       temp_buffer.begin() + frame_end);
        
        // CRC校验
        if (use_crc && data_length >= 2) {
            frame.crc = (frame.data[data_length - 2] << 8) | 
                        frame.data[data_length - 1];
            uint16_t calculated_crc = calculateCRC(frame.data.data(), data_length - 2);
            frame.crc_valid = (frame.crc == calculated_crc);
            
            if (frame.crc_valid && data_length >= 2) {
                frame.data.resize(data_length - 2);
            }
        } else {
            frame.crc = 0;
            frame.crc_valid = true;
        }
        
        frames.push_back(frame);
        pos = frame_end;
    }
    
    return frames;
}

// 异步处理线程
void ProtocolHandler::processThreadFunc() {
    while (running) {
        vector<uint8_t> data;
        
        {
            unique_lock<mutex> lock(buffer_mutex);
            if (data_queue.empty()) {
                data_cond.wait_for(lock, chrono::milliseconds(100));
                if (data_queue.empty()) continue;
            }
            
            data = data_queue.front();
            data_queue.pop();
        }
        
        if (!data.empty()) {
            processReceivedData(data);
        }
    }
}

// 启动异步处理
void ProtocolHandler::startAsyncProcessing() {
    if (!async_mode) {
        async_mode = true;
        running = true;
        process_thread = thread(&ProtocolHandler::processThreadFunc, this);
    }
}

// 停止异步处理
void ProtocolHandler::stopAsyncProcessing() {
    if (async_mode) {
        running = false;
        data_cond.notify_all();
        if (process_thread.joinable()) {
            process_thread.join();
        }
        async_mode = false;
    }
}

// 清空缓冲区
void ProtocolHandler::clearBuffer() {
    lock_guard<mutex> lock(buffer_mutex);
    rx_buffer.clear();
    while (!data_queue.empty()) {
        data_queue.pop();
    }
}