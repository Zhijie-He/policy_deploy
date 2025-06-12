#include <memory>
#include <csignal>
#include "utility/timer.h"
#include "utility/real/unitree_tools.h"
// unitree
// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

bool stop_flag = false;

void handle_sigint(int) {
    FRC_INFO("Shutting down simulate_robot...");
    stop_flag = true;
}

int main(int argc, char** argv) {
  signal(SIGINT, handle_sigint);
  ChannelFactory::Instance()->Init(0);

  auto state_pub = std::make_unique<ChannelPublisher<LowState_>>("rt/lowstate");
  state_pub->InitChannel();

  // 构造初始 LowState_
  // 构造并发布消息
  LowState_ fake_state;

  // IMU 数据
  fake_state.imu_state().rpy()[0] = 0.1;
  fake_state.imu_state().rpy()[1] = 0.2;
  fake_state.imu_state().rpy()[2] = 0.3;

  fake_state.imu_state().quaternion()[0] = 0.1;
  fake_state.imu_state().quaternion()[1] = 0.2;
  fake_state.imu_state().quaternion()[2] = 0.3;
  fake_state.imu_state().quaternion()[3] = 0.4;

  // 示例遥控器数据（按实际结构填充）
  auto& remote = fake_state.wireless_remote();
  // remote[0] = ...;  // 可填充 bitmap/float 数据


  uint64_t tick_counter = 0;
  Timer simulateRobotTimer(1); // 每秒 1 次

  while (!stop_flag) {
      fake_state.tick() = tick_counter++;
      // CRC 校验
      fake_state.crc() = unitree_tools::Crc32Core((uint32_t*)&fake_state, (sizeof(LowState_) >> 2) - 1);

      state_pub->Write(fake_state);
      FRC_INFO("[simulate_robot] Publishing fake LowState_ with tick " << tick_counter);
      simulateRobotTimer.wait();
  }

  return 0;
}

