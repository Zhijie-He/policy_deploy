#include "tasks/utils/mocap/MocapMsgSubscriber.hpp"
#include <thread>
#include <chrono>
#include <cstring>
#include "utility/logger.h"

MocapMsgSubscriber::MocapMsgSubscriber(float fps, int length)
    : fps_(fps), length_(length)
{
    participant_ = dds_create_participant(0, nullptr, nullptr);
    if (participant_ < 0) {
        FRC_ERROR("Failed to create DDS participant.");
        throw std::runtime_error("Failed to create DDS participant");
    }

    topic_ = dds_create_topic(participant_, &MocapMsg_desc, "topic/mocap", nullptr, nullptr);
    if (topic_ < 0) {
        FRC_ERROR("Failed to create DDS topic.");
        throw std::runtime_error("Failed to create DDS topic");
    }

    // 创建 Reader 的 QoS
    dds_qos_t* reader_qos = dds_create_qos();
    dds_qset_history(reader_qos, DDS_HISTORY_KEEP_LAST, MAX_SAMPLES);
    dds_qset_resource_limits(reader_qos, MAX_SAMPLES, DDS_LENGTH_UNLIMITED, DDS_LENGTH_UNLIMITED);

    // 直接用 participant 创建 reader，并传 QoS
    reader_ = dds_create_reader(participant_, topic_, reader_qos, nullptr);
    dds_delete_qos(reader_qos); 
    if (reader_ < 0) {
        FRC_ERROR("Failed to create DDS reader.");
        throw std::runtime_error("Failed to create DDS reader");
    }

    FRC_INFO("[MocapMsgSubscriber.Const] Waiting for Publisher....");
    dds_set_status_mask(reader_, DDS_SUBSCRIPTION_MATCHED_STATUS);

    while (true) {
        uint32_t status = 0;
        dds_get_status_changes(reader_, &status);
        if (status & DDS_SUBSCRIPTION_MATCHED_STATUS) {
            FRC_INFO("[MocapMsgSubscriber.Const] discover publisher!");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // at least qos 50 messages
    MocapMsg samples[MAX_SAMPLES];
    void* samples_void[MAX_SAMPLES];
    dds_sample_info_t infos[MAX_SAMPLES];
    for (int i = 0; i < MAX_SAMPLES; ++i) samples_void[i] = &samples[i];

    int ret;
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ret = dds_read(reader_, samples_void, infos, MAX_SAMPLES, MAX_SAMPLES);
    } while (ret < MAX_SAMPLES);

    msg_fps_ = samples[0].fps;
    msg_hands_ = samples[0].hands;
    FRC_INFO("[MocapMsgSubscriber.Const] msg_fps_: "<< msg_fps_ << ", msg_hands_: " << msg_hands_);
    FRC_INFO("[MocapMsgSubscriber.Const] created!");
}

MocapData MocapMsgSubscriber::subscribe() {
    MocapData result;

    MocapMsg samples[MAX_SAMPLES];
    void* samples_void[MAX_SAMPLES];
    dds_sample_info_t infos[MAX_SAMPLES];

    for (int i = 0; i < MAX_SAMPLES; ++i) samples_void[i] = &samples[i];

    int ret = dds_read(reader_, samples_void, infos, MAX_SAMPLES, MAX_SAMPLES);
    if (ret <= 0) {
        FRC_INFO("[MocapMsgSubscriber.subscribe] no new messages");
        return result;
    }
    
    // FRC_HIGHLIGHT("[MocapMsgSubscriber.subscribe] 本轮收到了 " << ret << " 条消息");
    for (int i = 1; i <= length_; ++i) {
        int idx = ret - 1 - static_cast<int>(i * (1.0 / fps_) * msg_fps_);
        if (idx < 0 || idx >= ret || !infos[idx].valid_data) {
            FRC_WARN("[MocapMsgSubscriber.subscribe] Skipping invalid idx: " << idx);
            continue;
        }

        const MocapMsg& msg = samples[idx];

        // root_rot: float[4]
        std::array<float, 4> root{};
        std::memcpy(root.data(), msg.root_rot, sizeof(float) * 4);
        result.root_rot.insert(result.root_rot.begin(), root);  

        // box_rot: float[2][4]
        std::array<std::array<float, 4>, 2> box{};
        for (int j = 0; j < 2; ++j)
            std::memcpy(box[j].data(), &msg.box_rot[j * 4], sizeof(float) * 4);
        result.box_rot.insert(result.box_rot.begin(), box);  

        // key_points: float[9][3]
        std::array<std::array<float, 3>, 9> keypoints{};
        for (int j = 0; j < 9; ++j)
            std::memcpy(keypoints[j].data(), &msg.key_points[j * 3], sizeof(float) * 3);
        result.key_points.insert(result.key_points.begin(), keypoints); 
    }

    return result;
}

MocapMsgSubscriber::~MocapMsgSubscriber() {
    dds_delete(participant_);
}

