#pragma once
#include <yaml-cpp/yaml.h>
#include "types/system_defined.h"
#include "core/BaseRobotConfig.h"
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "types/joystickTypes.h"
#include "simulator/playground.hpp"

class RaisimManager {
public:
  RaisimManager(std::shared_ptr<const BaseRobotConfig> cfg,
                 std::shared_ptr<jointCMD> jointCMDPtr,
                std::shared_ptr<robotStatus> robotStatusPtr);
  void initWorld();
  void loadRobotModel();
  void initState();
  void setupRobotProperties();
  void launchServer();
  void run();
  void updateRobotState();
  void integrate();
  void stop() { running_ = false; server_->killServer(); }
  void setUserInputPtr(char *key, JoystickData *joy) { joyPtr_ = joy; keyPtr_ = key; }

private:
  std::shared_ptr<const BaseRobotConfig> cfg_; 
  std::shared_ptr<jointCMD> jointCMDPtr_;
  std::shared_ptr<robotStatus> robotStatusPtr_;

  std::string robotName_;
  float control_dt_ = 2e-3;
  float simulation_dt_ = 5e-4;

  bool running_ = true;
  bool isStatesReady = false;
  bool isFixedBase = false;

  int gcDim_ = 0, gvDim_ = 0;
  int jointDim_ = 0;

  JoystickData* joyPtr_ = nullptr;
  char* keyPtr_ = nullptr;
  
  std::mutex state_lock_;
  std::mutex action_lock_;
  
  Eigen::VectorXd gc_init_, gv_init_;
  Eigen::VectorXd gc_, gv_, gf_, gv_prev_;
  Eigen::VectorXf pTarget, vTarget;
  Eigen::VectorXf tauCmd, fullTauCmd;
  Eigen::VectorXf jointPGain, jointDGain;

  raisim::World *world_;
  raisim::RaisimServer *server_;
  raisim::ArticulatedSystem *robot_;

  bool skillField = false;
  std::shared_ptr<MultiSkillField> _terrain;
};
