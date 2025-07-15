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

enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

namespace unitree_tools {
    uint32_t Crc32Core(uint32_t *ptr, uint32_t len);
    void create_zero_cmd(LowCmd_& cmd);
    void init_cmd_hg(LowCmd_& cmd, uint8_t mode_machine, Mode mode_pr);
    void print_lowcmd(const LowCmd_& cmd);
}