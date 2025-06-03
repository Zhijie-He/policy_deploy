#pragma once
#include <yaml-cpp/yaml.h>
#include "types/system_defined.h"
#include "core/RobotConfig.h"
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "types/joystickTypes.h"
#include "simulator/playground.hpp"

class RaisimManager {
public:
  RaisimManager(std::shared_ptr<const RobotConfig> cfg, GyroData *gyroPtr, jointStateData *motorInputPtr, jointTargetData *motorOutputPtr);
  void initWorld();
  void loadRobotModel();
  void initState();
  void setupRobotProperties();
  void launchServer();
  void run();
  void updateRobotState();
  void integrate();
  void reset();
  void getContactForce();
  void stop() { running_ = false; server_->killServer(); }
  void setUserInputPtr(char *key, JoystickData *joy) { joyPtr_ = joy; keyPtr_ = key; }

private:
  std::shared_ptr<const RobotConfig> cfg_; 
   
  std::string robotName_;
  float control_dt_ = 2e-3;
  float simulation_dt_ = 5e-4;
  
  
  bool running_ = true;
  bool isStatesReady = false;
  bool isFixedBase = false;
  bool isRopeHanging = false;
  float _ropeHeight = 0.9;
  
  int gcDim_ = 7, gvDim_ = 6;
  int jointDim_ = 0;
  
  std::unique_ptr<double> simTime;
  jointStateData *motorReadingBuf_;
  jointTargetData *motorCommandBuf_;
  GyroData *gyro_data_;
  JoystickData *joyPtr_ = nullptr;
  char *keyPtr_ = nullptr;
  
  std::mutex state_lock_;
  std::mutex action_lock_;
  
  Eigen::VectorXd gc_init_, gv_init_;
  Eigen::VectorXd gc_, gv_, gf_, gv_prev_;
  Eigen::VectorXf pTarget, vTarget;
  Eigen::VectorXf tauCmd, tauCmd_joint;
  Eigen::VectorXf jointPGain, jointDGain;
  Eigen::VectorXd contactForce;

  std::vector<size_t> shankBodyIdxs{0,0}; // toe 名称对应的 body/frame 索引
  std::vector<size_t> shankFrameIdxs{0,0};
  
  raisim::World *world_;
  raisim::RaisimServer *server_;
  raisim::ArticulatedSystem *robot_;

  bool skillField = false;
  std::shared_ptr<MultiSkillField> _terrain;
};
