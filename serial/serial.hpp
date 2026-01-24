#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <string>
#include <vector>
#include <cstdint>

class SerialPort {
private:
    int serial_fd;
    std::string port;
    int baudrate;
    
public:
    SerialPort();
    ~SerialPort();
    
    bool open(const std::string& port, int baudrate);
    void close();
    bool isOpen() const;
    
    // 发送数据
    int write(const std::string& data);
    int write(const uint8_t* data, size_t length);
    
    // 读取数据
    int read(uint8_t* buffer, size_t length, int timeout_ms = 100);
    int read(std::vector<uint8_t>& buffer, int timeout_ms = 100);
    std::vector<uint8_t> readAll(int timeout_ms = 100);
    
    // 获取可用字节数
    int available() const;
    
    // 清空缓冲区
    void flush();
};

#endif // SERIAL_HPP