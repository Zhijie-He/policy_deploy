// ===== include/sim2/BaseEnv.h =====
#pragma once

#include "core/BaseRobotConfig.h"
#include "types/joystickTypes.h"
#include "utility/data_buffer.h"

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
  virtual void setUserInputPtr(char* key, JoystickData* joy) { joyPtr_ = joy; keyPtr_ = key; }
  virtual void renderLoop() {}  
  virtual void run() {}
  virtual void integrate() {}
  
protected:
  std::shared_ptr<const BaseRobotConfig> cfg_;
  std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr_;
  std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr_;
  float control_dt_;
  bool running_ = true;

  JoystickData* joyPtr_ = nullptr;
  char* keyPtr_ = nullptr;

  int gcDim_, gvDim_, jointDim_;
  Eigen::VectorXd gc_, gv_;
  Eigen::VectorXf pTarget, vTarget;
  Eigen::VectorXf jointPGain, jointDGain;
};
