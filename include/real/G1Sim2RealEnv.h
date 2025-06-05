#pragma once
#include "core/BaseRobotConfig.h"
#include "types/system_defined.h"
#include "types/joystickTypes.h"

class G1Sim2RealEnv {
public:
  G1Sim2RealEnv(std::shared_ptr<const BaseRobotConfig> cfg, 
                const std::string& net_interface,
                jointCMD* jointCMDPtr,
                robotStatus* robotStatusPtr);

  void setUserInputPtr(char* key, JoystickData* joy) { joyPtr_ = joy; keyPtr_ = key; }
  void stop() { running_ = false;}

private:
  bool running_ = true;

  std::shared_ptr<const BaseRobotConfig> cfg_;
  robotStatus* robotStatusPtr_ = nullptr;
  jointCMD* jointCMDPtr_ = nullptr;

  JoystickData* joyPtr_ = nullptr;
  char* keyPtr_ = nullptr;
  float control_dt_;

  // unitree sdk
  uint8_t mode_machine_;
  // Mode mode_pr_;
  // std::shared_ptr<RemoteController> remote_controller_;
};