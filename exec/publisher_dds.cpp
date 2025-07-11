#include "dds/dds.h"
#include "tasks/utils/mocap/MocapMsg.h"  // C 接口结构体定义
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "[Publisher] Starting...\n";

    dds_entity_t participant = dds_create_participant(1, NULL, NULL);
    if (participant < 0) {
        std::cerr << "Failed to create participant\n";
        return -1;
    }

    // 创建 Topic
    dds_entity_t topic = dds_create_topic(
        participant, &MocapMsg_desc, "topic/mocap", NULL, NULL);
    if (topic < 0) {
        std::cerr << "Failed to create topic\n";
        return -1;
    }


    dds_qos_t* pub_qos = dds_create_qos();
    dds_qset_history(pub_qos, DDS_HISTORY_KEEP_LAST, 20);  // 缓存最近 32 条


    // 创建 Writer
    dds_entity_t writer = dds_create_writer(participant, topic, pub_qos, NULL);
    dds_delete_qos(pub_qos);
    if (writer < 0) {
        std::cerr << "Failed to create writer\n";
        return -1;
    }

    // 等待读者
    // dds_set_status_mask(writer, DDS_PUBLICATION_MATCHED_STATUS);
    // uint32_t status = 0;
    // while (!(status & DDS_PUBLICATION_MATCHED_STATUS)) {
    //     dds_get_status_changes(writer, &status);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }

    // 构造消息
    MocapMsg msg = {0};
    msg.fps = 50.0f;
    for (int i = 0; i < 4; ++i) msg.root_rot[i] = (i == 0) ? 1.0f : 0.0f;
    for (int i = 0; i < 8; ++i) msg.box_rot[i] = i;
    for (int i = 0; i < 27; ++i) msg.key_points[i] = float(i);
    msg.teleop = 1;
    for (int i = 0; i < 90; ++i) {
        msg.teleop_obs[i] = 0.1f * i;
    }
    msg.hands = 1;
    for (int i = 0; i < 21; ++i) {
        msg.left_hand_pos[i] = 0.1f * i;
        msg.right_hand_pos[i] = 0.2f * i;
    }
    for (int i = 0; i < 4; ++i) {
        msg.left_wrist_rot[i] = (i == 3) ? 1.0f : 0.0f;
        msg.right_wrist_rot[i] = (i == 3) ? 1.0f : 0.0f;
    }
    msg.retarget = 1;
    for (int i = 0; i < 14; ++i) msg.hands_joints[i] = float(i);
    for (int i = 0; i < 3; ++i) {
        msg.left_wrist_joints[i] = 0.1f * i;
        msg.right_wrist_joints[i] = 1.0f + 0.1f * i;
    }

    int j = 0;
    while (true)
    {   
        j++;
        // for (int i = 0; i < 8; ++i) msg.box_rot[i] = j;
        std::cout << "[Publisher] Sending MocapMsg frame " << j << "\n";
        if (dds_write(writer, &msg) < 0) {
            std::cerr << "Failed to write data\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    dds_delete(participant);
    std::cout << "[Publisher] Done.\n";
    return 0;
}
