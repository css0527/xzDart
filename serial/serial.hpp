#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <string>

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
    int write(const std::string& data);
    int write(const uint8_t* data, size_t length);
};

#endif // SERIAL_HPP