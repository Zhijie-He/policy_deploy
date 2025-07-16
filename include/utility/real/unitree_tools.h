// utility/real/unitree_tools.h
#pragma once

#include <cstdint>
// unitree
// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

// hands
#include <unitree/idl/hg/HandCmd_.hpp>
#include <unitree/idl/hg/HandState_.hpp>

using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

#define MOTOR_MAX 7
#define SENSOR_MAX 9

enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

class RIS_Mode {
public:
    RIS_Mode(uint8_t id = 0, uint8_t status = 0x01, uint8_t timeout = 0)
        : id_(id & 0x0F), status_(status & 0x07), timeout_(timeout & 0x01) {}

    uint8_t modeToUint8() const {
        uint8_t mode = 0;
        mode |= (id_ & 0x0F);
        mode |= (status_ & 0x07) << 4;
        mode |= (timeout_ & 0x01) << 7;
        return mode;
    }
private:
    uint8_t id_;
    uint8_t status_;
    uint8_t timeout_;
};

namespace unitree_tools {
    uint32_t Crc32Core(uint32_t *ptr, uint32_t len);
    void create_zero_cmd(LowCmd_& cmd);
    void init_cmd_hg(LowCmd_& cmd, uint8_t mode_machine, Mode mode_pr);
    void print_lowcmd(const LowCmd_& cmd);
}