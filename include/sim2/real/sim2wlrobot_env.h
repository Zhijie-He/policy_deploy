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
};
