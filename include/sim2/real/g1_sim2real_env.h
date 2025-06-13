// ===== include/sim2/real/g1_sim2real_env.h =====
#pragma once
#include <memory>
#include "types/system_defined.h"
#include "sim2/base_env.h"
#include "state_machine/StateMachine.h"

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

enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

class G1Sim2RealEnv : public BaseEnv {
public:
  G1Sim2RealEnv(const std::string& net_interface,
            std::shared_ptr<const BaseRobotConfig> cfg,
            std::shared_ptr<StateMachine> state_machine);   

  G1Sim2RealEnv(const std::string& net_interface,
          std::shared_ptr<const BaseRobotConfig> cfg,
          std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr,
          std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr);   

  void stop() override { running_ = false;}
  void LowStateHandler(const void *message);
  void initState();
  void updateRobotState();
  void waitForLowState();

  void sendCmd(LowCmd_& cmd);
  void zeroTorqueState() override;
  void moveToDefaultPos() override;
  void defaultPosState() override;
  void run() override;

private:
  std::shared_ptr<StateMachine> state_machine_ = nullptr;
  // 对应构造参数的成员变量
  std::string net_interface_;
  int counter_ = 0;

  // unitree sdk
  uint8_t mode_machine_;
  Mode mode_pr_;
  LowCmd_ low_cmd_;
  LowState_ low_state_;
  
  // publisher
  std::unique_ptr<ChannelPublisher<LowCmd_>> lowcmd_publisher_;
  // subscriber
  std::unique_ptr<ChannelSubscriber<LowState_>> lowstate_subscriber_;

  unitree::common::ThreadPtr command_writer_ptr_, control_thread_ptr_;
};
