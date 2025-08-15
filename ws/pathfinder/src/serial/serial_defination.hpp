#pragma once

#include "../state_machine/state_defines.hpp"
#include <cstdint>

namespace engineering_match::serial_package {

struct __attribute__((packed)) PackageToSend {
    uint8_t package_header = { 0xA5 };
    uint8_t command_id = { 0x00 };
    uint8_t data = { 0x00 };
};

struct __attribute__((packed)) PackageToReceive {
    uint8_t package_header = { 0xA5 };
    uint8_t command_id;
};

enum class EmbeddedCMD {
    IDLE = 0,
    FMOVE = 1,
    BMOVE = 2,
    LMOVE = 3,
    RMOVE = 4,
    LFMOVE = 5,
    RFMOVE = 6,
    LBMOVE = 7,
    RBMOVE = 8,
    LROT = 9,
    RROT = 10,
    STOP = 11,
    FCLAMPONE = 12, // 后夹子
    FCLAMPTWO = 13,
    FCLAMPUP = 14,
    BCLAMP = 15, // 前夹子（一次五个的）
    FPLACE = 16, // 后放一个
    BPLACE = 17 // 前放一个
};

using namespace state_machine;

// 由于下层板对正方向定义不一致，重新对应命令
inline std::unordered_map<CommandType, EmbeddedCMD> recast_cmd = {
    { CommandType::IDLE, EmbeddedCMD::IDLE },
    { CommandType::FMOVE, EmbeddedCMD::BMOVE },
    { CommandType::BMOVE, EmbeddedCMD::FMOVE },
    { CommandType::LMOVE, EmbeddedCMD::RMOVE },
    { CommandType::RMOVE, EmbeddedCMD::LMOVE },
    { CommandType::LFMOVE, EmbeddedCMD::RBMOVE },
    { CommandType::RFMOVE, EmbeddedCMD::LBMOVE },
    { CommandType::LBMOVE, EmbeddedCMD::RFMOVE },
    { CommandType::RBMOVE, EmbeddedCMD::LFMOVE },
    { CommandType::LROT, EmbeddedCMD::LROT },
    { CommandType::RROT, EmbeddedCMD::RROT },
    { CommandType::FCLAMPONE, EmbeddedCMD::FCLAMPONE },
    { CommandType::FCLAMPTWO, EmbeddedCMD::FCLAMPTWO },
    { CommandType::FCLAMPUP, EmbeddedCMD::FCLAMPUP },
    { CommandType::BCLAMP, EmbeddedCMD::BCLAMP },
    { CommandType::FPLACE, EmbeddedCMD::FPLACE },
    { CommandType::BPLACE, EmbeddedCMD::BPLACE },
};
}