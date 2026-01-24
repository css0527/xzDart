#include "crc.hpp"

uint16_t crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF; // Initial value for CRC-16-Modbus
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001; // Polynomial 0xA001
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}