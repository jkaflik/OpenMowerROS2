#pragma once

#include <iostream>
#include <vector>
#include <cstdint>

constexpr uint16_t POLYNOMIAL = 0x1021;
uint16_t crc16Table[256];

void generateCRCTable() {
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (uint16_t j = 8; j > 0; j--) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ POLYNOMIAL;
            else
                crc <<= 1;
        }
        crc16Table[i] = crc;
    }
}

uint16_t calculateCRC16(const std::vector<uint8_t>& data) {
    uint16_t crcValue = 0xFFFF; // Standard initial value for CRC-16-CCITT

    for (uint8_t byte : data) {
        uint8_t tableIndex = (crcValue >> 8) ^ byte;
        crcValue = crc16Table[tableIndex] ^ (crcValue << 8);
    }

    return crcValue; // In this example, we don't invert the bits at the end
}
