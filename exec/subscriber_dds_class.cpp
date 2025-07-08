#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include "tasks/utils/mocap/MocapMsgSubscriber.hpp"

// ------------------ 主函数示例 ------------------
int main() {
    MocapMsgSubscriber subscriber(30.0f, 9);

    while (true) {
        MocapData data = subscriber.subscribe();

        std::cout << "======= Mocap Frame Received =======" << std::endl;
        for (size_t i = 0; i < data.root_rot.size(); ++i) {
            std::cout << "Frame [" << i << "]" << std::endl;

            // 1. 打印 root_rot
            std::cout << "  root_rot: [ ";
            for (float x : data.root_rot[i]) {
                std::cout << std::fixed << std::setprecision(3) << x << " ";
            }
            std::cout << "]" << std::endl;

            // 2. 打印 box_rot
            std::cout << "  box_rot: [ ";
            for (const auto& quat : data.box_rot[i]) {
                std::cout << "[ ";
                for (float q : quat) std::cout << std::fixed << std::setprecision(3) << q << " ";
                std::cout << "] ";
            }
            std::cout << "]" << std::endl;

            // 3. 打印 key_points
            std::cout << "  key_points:" << std::endl;
            for (size_t j = 0; j < data.key_points[i].size(); ++j) {
                const auto& point = data.key_points[i][j];
                std::cout << "    " << j << ": ("
                          << std::fixed << std::setprecision(2)
                          << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
            }

            std::cout << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 每秒采样一次
    }
}

