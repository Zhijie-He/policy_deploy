#pragma once
#include "core/BaseRobotConfig.h"
#include "types/system_defined.h"
#include "types/joystickTypes.h"
#include <memory>
#include "hardware/listener.h"

// unitree
// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

class G1Sim2RealEnv {
public:
  G1Sim2RealEnv(std::shared_ptr<const BaseRobotConfig> cfg, 
                const std::string& net_interface,
                std::shared_ptr<jointCMD> jointCMDPtr,
                std::shared_ptr<robotStatus> robotStatusPtr);

  void setUserInputPtr(char* key, JoystickData* joy, std::shared_ptr<Listener> listener) { keyPtr_ = key; joyPtr_ = joy; listenerPtr_ = listener; }
  void stop() { running_ = false;}
  void LowStateHandler(const void *message);
  void waitForLowState();
  void simulateRobot();
  void zeroTorqueState();

private:
  bool running_ = true;

  std::shared_ptr<const BaseRobotConfig> cfg_;
  std::shared_ptr<jointCMD> jointCMDPtr_;
  std::shared_ptr<robotStatus> robotStatusPtr_;

  JoystickData* joyPtr_ = nullptr;
  char* keyPtr_ = nullptr;
  std::shared_ptr<Listener> listenerPtr_;

  float control_dt_;

  // unitree sdk
  uint8_t mode_machine_;
  Mode mode_pr_;
  LowCmd_ low_cmd_;
  LowState_ low_state_;

  // publisher
  std::unique_ptr<ChannelPublisher<LowCmd_>> lowcmd_publisher_;
  // subscriber
  std::unique_ptr<ChannelSubscriber<LowState_>> lowstate_subscriber_;

  ThreadPtr command_writer_ptr_, control_thread_ptr_;
};