#pragma once

#include <cstdint>

namespace engineering_match::serial_package {

struct __attribute__((packed)) PackageToSend {
    uint8_t package_header = { 0xA5 };
    uint8_t command_id = { 0x00 };
    float move_data[3] = { 0.0f, 0.0f, 0.0f };
};

struct __attribute__((packed)) PackageToReceive {
    uint8_t package_header = { 0xA5 };
    uint8_t command_id;
};

}