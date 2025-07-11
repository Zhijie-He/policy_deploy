#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include "tasks/utils/mocap/MocapMsgSubscriber.h"

// ------------------ 主函数示例 ------------------
int main() {
    MocapMsgSubscriber subscriber(30.0f, 9);

    while (true) {
        auto hands_data = subscriber.subscribeHands();

        std::cout << "======= Mocap Hands Frame Received =======" << std::endl;

        // 1. 打印 hands_joints (2x7) 展平
        const auto& hands_flat = hands_data.at("hands_joints");
        std::cout << "hands_joints (2x7):" << std::endl;
        for (int i = 0; i < 2; ++i) {
            std::cout << "  Hand " << i << ": [ ";
            for (int j = 0; j < 7; ++j) {
                std::cout << std::fixed << std::setprecision(3) << hands_flat[i * 7 + j] << " ";
            }
            std::cout << "]" << std::endl;
        }

        // 2. 打印 left_wrist_joints
        const auto& left = hands_data.at("left_wrist_joints");
        std::cout << "left_wrist_joints: [ ";
        for (int i = 0; i < left.size(); ++i) {
            std::cout << std::fixed << std::setprecision(3) << left[i] << " ";
        }
        std::cout << "]" << std::endl;

        // 3. 打印 right_wrist_joints
        const auto& right = hands_data.at("right_wrist_joints");
        std::cout << "right_wrist_joints: [ ";
        for (int i = 0; i < right.size(); ++i) {
            std::cout << std::fixed << std::setprecision(3) << right[i] << " ";
        }
        std::cout << "]" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 每秒打印一次
    }
    
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

