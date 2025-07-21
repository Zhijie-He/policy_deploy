// ===== include/sim2/real/g1_sim2real_env.h =====
#pragma once
#include <memory>
#include "types/system_defined.h"
#include "sim2/base_env.h"
#include "utility/real/unitree_tools.h"

class G1Sim2RealEnv : public BaseEnv {
public:
  G1Sim2RealEnv(const std::string& net_interface,
            std::shared_ptr<const BaseRobotConfig> cfg,
            std::shared_ptr<StateMachine> state_machine);   

  void stop() override { running_ = false;}
  void LowStateHandler(const void *message);
  void updateRobotState();
  void waitForLowState();

  void sendCmd(LowCmd_& cmd);
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

  // unitree sdk
  uint8_t mode_machine_;
  Mode mode_pr_;
  
  LowCmd_ low_cmd_;
  LowState_ low_state_;

  DataBuffer<LowState_> low_state_buffer_;

  // publisher
  std::unique_ptr<ChannelPublisher<LowCmd_>> lowcmd_publisher_;
  // subscriber
  std::unique_ptr<ChannelSubscriber<LowState_>> lowstate_subscriber_;
};
