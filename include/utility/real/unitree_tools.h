// utility/real/unitree_tools.h
#pragma once
#include <cstdint>

namespace unitree_tools {
    uint32_t Crc32Core(uint32_t *ptr, uint32_t len);
}