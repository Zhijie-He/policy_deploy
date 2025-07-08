#include "dds/dds.h"
#include "tasks/utils/mocap/MocapMsg.h"  // IDL 生成的结构体定义
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    dds_entity_t participant = dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr);
    if (participant < 0) {
        std::cerr << "Failed to create participant\n";
        return -1;
    }

    dds_entity_t topic = dds_create_topic(
        participant,
        &MocapMsg_desc,            // 从 MocapMsg.h 中获得
        "topic/mocap",
        nullptr, nullptr);
    if (topic < 0) {
        std::cerr << "Failed to create topic\n";
        return -1;
    }

    constexpr int MAX_SAMPLES = 50;

    // ✅ 创建并配置 QoS
    dds_qos_t* sub_qos = dds_create_qos();
    dds_qset_history(sub_qos, DDS_HISTORY_KEEP_LAST, MAX_SAMPLES);
    dds_qset_resource_limits(sub_qos, MAX_SAMPLES, DDS_LENGTH_UNLIMITED, DDS_LENGTH_UNLIMITED);

    // ✅ 使用 QoS 创建 reader
    dds_entity_t reader = dds_create_reader(participant, topic, sub_qos, nullptr);
    dds_delete_qos(sub_qos);  // 用完立即释放
    if (reader < 0) {
        std::cerr << "Failed to create reader\n";
        return -1;
    }

    std::cout << "[Subscriber] 等待 Publisher 匹配...\n";
    dds_set_status_mask(reader, DDS_SUBSCRIPTION_MATCHED_STATUS);

    while (true) {
        uint32_t status = 0;
        dds_get_status_changes(reader, &status);

        if (status & DDS_SUBSCRIPTION_MATCHED_STATUS) {
            std::cout << "✅ Publisher 已匹配上\n";
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    MocapMsg samples[MAX_SAMPLES];
    void* samples_void[MAX_SAMPLES];
    dds_sample_info_t infos[MAX_SAMPLES];

    for (int i = 0; i < MAX_SAMPLES; ++i)
        samples_void[i] = &samples[i];

    while (true) {
        int ret = dds_read(reader, samples_void, infos, MAX_SAMPLES, MAX_SAMPLES);
        if (ret > 0) {
            std::cout << "📥 本轮收到了 " << ret << " 条消息\n";
            for (int i = 0; i < ret; ++i) {
                if (!infos[i].valid_data) continue;
                const MocapMsg& msg = samples[i];
                std::cout << "  - Msg[" << i << "] FPS: " << msg.fps << "\n";
                std::cout << "Root Rot: [";
                for (int i = 0; i < 4; ++i) std::cout << msg.root_rot[i] << (i < 3 ? ", " : "]\n");
            }
        } else {
            std::cout << "（暂无新消息）\n";
        }

        dds_sleepfor(DDS_MSECS(1000));
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    dds_delete(participant);
    return 0;

    while (true) {
        MocapMsg samples[1];
        void* samples_void[1] = { samples };
        dds_sample_info_t infos[1];

        int ret = dds_take(reader, samples_void, infos, 1, 1);
        if (ret > 0 && infos[0].valid_data) {
            const MocapMsg& msg = samples[0];
            std::cout << "====== [Subscriber] Got MocapMsg ======\n";

            std::cout << "FPS: " << msg.fps << "\n";

            std::cout << "Root Rot: [";
            for (int i = 0; i < 4; ++i) std::cout << msg.root_rot[i] << (i < 3 ? ", " : "]\n");

            std::cout << "Box Rot: [";
            for (int i = 0; i < 8; ++i) std::cout << msg.box_rot[i] << (i < 7 ? ", " : "]\n");

            std::cout << "Key Points (9x3): [";
            for (int i = 0; i < 27; ++i) std::cout << msg.key_points[i] << (i < 26 ? ", " : "]\n");

            std::cout << "Hands: " << static_cast<int>(msg.hands) << "\n";

            std::cout << "Left Hand Pos (7x3): [";
            for (int i = 0; i < 21; ++i) std::cout << msg.left_hand_pos[i] << (i < 20 ? ", " : "]\n");

            std::cout << "Right Hand Pos (7x3): [";
            for (int i = 0; i < 21; ++i) std::cout << msg.right_hand_pos[i] << (i < 20 ? ", " : "]\n");

            std::cout << "Left Wrist Rot: [";
            for (int i = 0; i < 4; ++i) std::cout << msg.left_wrist_rot[i] << (i < 3 ? ", " : "]\n");

            std::cout << "Right Wrist Rot: [";
            for (int i = 0; i < 4; ++i) std::cout << msg.right_wrist_rot[i] << (i < 3 ? ", " : "]\n");

            std::cout << "Retarget: " << static_cast<int>(msg.retarget) << "\n";

            std::cout << "Hands Joints (2x7): [";
            for (int i = 0; i < 14; ++i) std::cout << msg.hands_joints[i] << (i < 13 ? ", " : "]\n");

            std::cout << "Left Wrist Joints (3): [";
            for (int i = 0; i < 3; ++i) std::cout << msg.left_wrist_joints[i] << (i < 2 ? ", " : "]\n");

            std::cout << "Right Wrist Joints (3): [";
            for (int i = 0; i < 3; ++i) std::cout << msg.right_wrist_joints[i] << (i < 2 ? ", " : "]\n");

            std::cout << "========================================\n";
        }

        dds_sleepfor(DDS_MSECS(50));
    }


    dds_delete(participant);
    return 0;
}


