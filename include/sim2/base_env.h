// ===== include/sim2/BaseEnv.h =====
#pragma once

#include "config/BaseRobotConfig.h"
#include "types/joystickTypes.h"
#include "utility/data_buffer.h"
#include "hardware/listener.h"
#include "types/CustomTypes.h"
#include "state_machine/StateMachine.h"
#include "utility/logger.h"
#include "utility/timer.h"
#include "utility/tools.h"

class BaseEnv {
public:
  BaseEnv(std::shared_ptr<const BaseRobotConfig> cfg,
          std::shared_ptr<StateMachine> state_machine);
        
  virtual ~BaseEnv() = default;
  void initState();
  virtual void stop() { running_ = false;}  
  virtual void setHeadless(bool) {}
  virtual void setUserInputPtr(std::shared_ptr<Listener> listener, char* key, JoystickData* joy) {listenerPtr_ = listener; keyPtr_ = key; joyPtr_ = joy;}
  virtual void run() {}

  virtual void zeroTorqueState() {}
  virtual void moveToTransferPos() {}
  virtual void transferPosState() {}
  virtual void moveToDefaultPos() {}
  virtual void defaultPosState() {}


  virtual void applyAction(const jointCMD& cmd) = 0;  // 子类实现控制信号的应用（sim/real）
  virtual bool isRunning() { return running_; } // 可被 override
  void runControlLoop(); // 公共控制主循环（供子类调用）

protected:
  std::shared_ptr<StateMachine> state_machine_ = nullptr;
  std::shared_ptr<const BaseRobotConfig> cfg_;
  std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr_;
  std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr_;
  
  float control_dt_;
  bool running_ = true;
  
  // listener related
  std::shared_ptr<Listener> listenerPtr_;
  char* keyPtr_ = nullptr;
  JoystickData* joyPtr_ = nullptr;

  int gcDim_, gvDim_, jointDim_, actuatorDim_;
  Eigen::VectorXf gc_, gv_, joint_torques_;
  Eigen::VectorXf pTarget, vTarget;
  Eigen::VectorXf jointPGain, jointDGain;
  Eigen::VectorXf tauCmd;

  std::mutex state_lock_;
  std::mutex action_lock_;

  int run_count=0;
  double run_sum_us=0;
  double run_sum_sq_us=0;
};
