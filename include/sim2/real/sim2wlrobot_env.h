// ===== include/sim2/real/sim2wlrobot_env.h =====
#pragma once
#include <memory>
#include "types/system_defined.h"
#include "sim2/base_env.h"

#include <wlrobot/robot/channel/channel_subscriber.hpp>
#include <wlrobot/robot/channel/channel_publisher.hpp>
#include <wlrobot/robot/low_level/low_state_aggregator.hpp>

#include <wlrobot/idl/hd/IMUStatePubSubTypes.hpp>
#include <wlrobot/idl/hd/MotorStatesPubSubTypes.hpp>
#include <wlrobot/idl/hd/MotorCmdsPubSubTypes.hpp>
#include <wlrobot/idl/hd/LowStatePubSubTypes.hpp>

using namespace wlrobot::robot;
using namespace wlrobot::msg;

class Sim2WlRobotEnv : public BaseEnv {
public:
  Sim2WlRobotEnv(const std::string& net_interface,
                 std::shared_ptr<const BaseRobotConfig> cfg,
                 std::shared_ptr<StateMachine> state_machine);   

  void stop() override { running_ = false;}
  void LowStateHandler(const LowState& message);
  void updateRobotState();
  void waitForLowState();

  void sendCmd(MotorCmds& cmd);
  void zeroTorqueState() override;
  void moveToDefaultPos() override;
  void defaultPosState() override;
  void run() override;

  bool isRunning() const override; 
  void applyAction(const jointCMD& cmd) override;  

private:
  // 对应构造参数的成员变量
  std::string net_interface_;
  int counter_ = 0;
  
  MotorCmds low_cmd_;
  LowState low_state_;

  DataBuffer<LowState> low_state_buffer_;

  // publisher
  std::unique_ptr<ChannelPublisher<MotorCmds>> lowcmd_publisher_;
  // subscriber
  std::unique_ptr<ChannelSubscriber<LowState>> lowstate_subscriber_;

  //related to calibartion
  std::array<float, 4> abad_sign_  = { 1.0f,  1.0f, -1.0f, -1.0f };
  std::array<float, 4> hip_sign_   = {-1.0f,  1.0f, -1.0f,  1.0f };
  std::array<float, 4> knee_sign_  = {-1.0f,  1.0f, -1.0f,  1.0f };
  std::array<std::array<float, 3>, 4> zero_offset_ = {{
      { 0.882785f,  1.56933f,  -3.15066f },
      {-0.245677f, -0.873517f,  2.25232f },
      { 0.169374f,  1.31086f,  -2.8007f  },
      { 0.422843f, -0.535587f,  2.45248f }
  }};
  std::array<int, 12> real2sim_dof_map_ = { 3,4,5, 0,1,2, 9,10,11, 6,7,8 };
};
