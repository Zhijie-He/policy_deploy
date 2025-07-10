#pragma once

#include <vector>
#include <array>

// 通用运动捕捉数据结构体
struct MocapData {
    std::vector<std::array<float, 4>> root_rot;     // 四元数旋转（length, 4）
    std::vector<std::array<std::array<float, 4>, 2>> box_rot;      // 2 个物体旋转（length, 2, 4）
    std::vector<std::array<std::array<float, 3>, 9>> key_points;   // 关键点坐标（length, 9, 3）
    std::vector<std::array<float, 90>> teleop_obs;   // teleop obs（length, 90）
};

// struct MocapResult {
//     std::vector<std::vector<std::array<float, 3>>> motion_state;  // [T][30][3]
//     std::vector<std::vector<float>> mocap_obs;                    // [T][90]
// };

struct MocapResult {
    std::vector<std::vector<float>> motion_state;  // [T][90]
    std::vector<std::vector<float>> teleop_obs;     // [T][90]
};
