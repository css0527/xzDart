#include "serial.hpp"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

using namespace std;

SerialPort::SerialPort() : serial_fd(-1) {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port, int baudrate) {
    this->port = port;
    this->baudrate = baudrate;
    
    serial_fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd < 0) {
        cerr << "Failed to open serial port: " << port << endl;
        return false;
    }
    
    struct termios options;
    tcgetattr(serial_fd, &options);
    
    // 设置波特率
    speed_t br;
    switch(baudrate) {
        case 9600: br = B9600; break;
        case 19200: br = B19200; break;
        case 38400: br = B38400; break;
        case 57600: br = B57600; break;
        case 115200: br = B115200; break;
        case 230400: br = B230400; break;
        default: br = B115200; break;
    }
    cfsetispeed(&options, br);
    cfsetospeed(&options, br);
    
    // 8N1
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    // 启用接收，忽略调制解调器控制线
    options.c_cflag |= (CLOCAL | CREAD);
    
    // 禁用流控制
    options.c_cflag &= ~CRTSCTS;
    
    // 原始输入模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // 原始输出模式
    options.c_oflag &= ~OPOST;
    
    // 设置超时
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10; // 1秒超时
    
    tcsetattr(serial_fd, TCSANOW, &options);
    
    cout << "Serial port opened: " << port << " at " << baudrate << " baud" << endl;
    return true;
}

void SerialPort::close() {
    if (serial_fd >= 0) {
        ::close(serial_fd);
        serial_fd = -1;
    }
}

bool SerialPort::isOpen() const {
    return serial_fd >= 0;
}

int SerialPort::write(const std::string& data) {
    if (!isOpen()) return -1;
    return ::write(serial_fd, data.c_str(), data.length());
}

int SerialPort::write(const uint8_t* data, size_t length) {
    if (!isOpen()) return -1;
    return ::write(serial_fd, data, length);
}