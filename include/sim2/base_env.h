// ===== include/sim2/BaseEnv.h =====
#pragma once

#include "config/BaseRobotConfig.h"
#include "types/joystickTypes.h"
#include "utility/data_buffer.h"
#include "hardware/listener.h"
#include "types/CustomTypes.h"

class BaseEnv {
public:
  BaseEnv(std::shared_ptr<const BaseRobotConfig> cfg,
          std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr,
          std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr)
      : cfg_(cfg),
        jointCMDBufferPtr_(jointCMDBufferPtr),
        robotStatusBufferPtr_(robotStatusBufferPtr),
        control_dt_(cfg->getPolicyDt()) {}
        
  virtual ~BaseEnv() = default;
  virtual void stop() { running_ = false;}  
  virtual void setUserInputPtr(std::shared_ptr<Listener> listener, char* key, JoystickData* joy) {listenerPtr_ = listener; keyPtr_ = key; joyPtr_ = joy;}
  virtual void run() {}

  virtual void zeroTorqueState() {}
  virtual void moveToDefaultPos() {}
  virtual void defaultPosState() {}
  
protected:
  std::shared_ptr<const BaseRobotConfig> cfg_;
  std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr_;
  std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr_;
  float control_dt_;
  bool running_ = true;
  
  // listener related
  std::shared_ptr<Listener> listenerPtr_;
  char* keyPtr_ = nullptr;
  JoystickData* joyPtr_ = nullptr;

  int gcDim_, gvDim_, jointDim_;
  Eigen::VectorXd gc_, gv_, joint_torques_;
  Eigen::VectorXf pTarget, vTarget;
  Eigen::VectorXf jointPGain, jointDGain;

  std::mutex state_lock_;
  std::mutex action_lock_;

  std::string mode_;
  std::string track_;
  std::vector<std::string> track_list_;
  std::shared_ptr<CustomTypes::MocapConfig> mocap_cfg_;
  std::shared_ptr<CustomTypes::VlaConfig> vla_cfg_;
};
