#ifndef PROTOCOL_HANDLER_HPP
#define PROTOCOL_HANDLER_HPP

#include <cstdint>
#include <vector>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <thread>

// 接收到的完整数据帧
struct ReceivedFrame {
    std::vector<uint8_t> header;
    std::vector<uint8_t> data;
    std::vector<uint8_t> footer;
    uint16_t crc;
    bool crc_valid;
};

// 发送数据包
struct SendPacket {
    std::vector<uint8_t> header;
    std::vector<uint8_t> data;
    std::vector<uint8_t> footer;
    bool with_crc;
};

// 回调函数类型定义
using FrameReceivedCallback = std::function<void(const ReceivedFrame&)>;
using RawDataCallback = std::function<void(const std::vector<uint8_t>&)>;

class ProtocolHandler {
private:
    // 帧头帧尾配置
    std::vector<uint8_t> rx_header;      // 接收帧头
    std::vector<uint8_t> rx_footer;      // 接收帧尾
    std::vector<uint8_t> tx_header;      // 发送帧头
    std::vector<uint8_t> tx_footer;      // 发送帧尾
    
    bool use_crc;                        // 是否使用CRC校验
    bool async_mode;                     // 是否异步模式
    
    // 接收缓冲区
    std::vector<uint8_t> rx_buffer;
    size_t max_buffer_size;
    
    // 异步处理相关
    std::thread process_thread;
    bool running;
    std::mutex buffer_mutex;
    std::condition_variable data_cond;
    std::queue<std::vector<uint8_t>> data_queue;
    
    // 回调函数
    FrameReceivedCallback frame_callback;
    RawDataCallback raw_callback;
    
    // CRC计算
    uint16_t calculateCRC(const uint8_t* data, size_t length);
    
    // 数据处理
    void processReceivedData(const std::vector<uint8_t>& data);
    void processThreadFunc();
    
    // 帧匹配
    bool matchHeader(const std::vector<uint8_t>& data, size_t pos);
    bool matchFooter(const std::vector<uint8_t>& data, size_t pos);
    
public:
    ProtocolHandler();
    ~ProtocolHandler();
    
    // 配置设置
    void setReceiveHeader(const std::vector<uint8_t>& header);
    void setReceiveFooter(const std::vector<uint8_t>& footer);
    void setTransmitHeader(const std::vector<uint8_t>& header);
    void setTransmitFooter(const std::vector<uint8_t>& footer);
    void setUseCRC(bool use) { use_crc = use; }
    void setMaxBufferSize(size_t size) { max_buffer_size = size; }
    
    // 回调设置
    void setFrameReceivedCallback(FrameReceivedCallback callback);
    void setRawDataCallback(RawDataCallback callback);
    
    // 数据处理
    void feedData(const uint8_t* data, size_t length);
    void feedData(const std::vector<uint8_t>& data);
    
    // 数据打包
    std::vector<uint8_t> packData(const std::vector<uint8_t>& data, bool include_crc = true);
    std::vector<uint8_t> packData(const uint8_t* data, size_t length, bool include_crc = true);
    
    // 数据解包（同步模式）
    std::vector<ReceivedFrame> unpackData(const std::vector<uint8_t>& data);
    
    // 异步处理控制
    void startAsyncProcessing();
    void stopAsyncProcessing();
    
    // 清空缓冲区
    void clearBuffer();
};
#endif // PROTOCOL_HANDLER_HPP