// ===== include/sim2/real/g1_sim2real_env.h =====
#pragma once
#include <memory>
#include "types/system_defined.h"
#include "sim2/base_env.h"
#include "tasks/utils/mocap/Keypoints.h"
// unitree related
#include "utility/real/unitree_tools.h"


class G1Sim2RealEnv : public BaseEnv {
public:
  G1Sim2RealEnv(const std::string& net_interface,
            std::shared_ptr<const BaseRobotConfig> cfg,
            const std::string& hands_type,
            std::shared_ptr<StateMachine> state_machine);   

  void initWorld() override;
  void stop() override { running_ = false;}
  void LowStateHgHandler(const void *message);
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
  std::string net_interface_;
  
  // unitree sdk
  uint8_t mode_machine_;
  Mode mode_pr_;
  LowCmd_ low_cmd_;
  LowState_ low_state_;
  DataBuffer<LowState_> low_state_buffer_;
  std::unique_ptr<ChannelPublisher<LowCmd_>> lowcmd_publisher_;         // publisher
  std::unique_ptr<ChannelSubscriber<LowState_>> lowstate_subscriber_;   // subscriber

  // hands related
  HandCmd_ leftHand_cmd_;
  HandState_ leftHand_state_;
  std::unique_ptr<ChannelPublisher<HandCmd_>> leftHandcmd_publisher_;         // publisher
  std::unique_ptr<ChannelSubscriber<HandCmd_>> leftHandstate_subscriber_;         // subscriber
  
  HandCmd_ rightHand_cmd_;
  HandState_ rightHand_state_;
  std::unique_ptr<ChannelPublisher<HandCmd_>> rightHandcmd_publisher_;         // publisher
  std::unique_ptr<ChannelSubscriber<HandCmd_>> rightHandstate_subscriber_;         // subscriber

  // visualization 
  dds_entity_t keypoints_publisher_; 
  mujoco_visualization_KeypointsMsg keypoints_msg_;
};
