#pragma once

#include <vector>
#include <array>
#include <unordered_map>
#include <Eigen/Dense>
#include "dds/dds.h"
#include "tasks/utils/mocap/MocapMsg.h"  // 包含由 IDL 生成的结构体定义
#include "tasks/utils/mocap/MocapData.h" 

class MocapMsgSubscriber {
public:
    MocapMsgSubscriber(float fps, int length);
    ~MocapMsgSubscriber();

    MocapData subscribe();
    std::unordered_map<std::string, Eigen::VectorXf> subscribeHands();

private:
    dds_entity_t participant_;
    dds_entity_t topic_;
    dds_entity_t reader_;

    float fps_;
    int length_;
    float msg_fps_;
    int msg_hands_;
    int MAX_SAMPLES = 50;
};

