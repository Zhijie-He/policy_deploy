#pragma once

#include <yaml-cpp/yaml.h>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <memory>
#include <mutex>
#include <Eigen/Core>
#include <vector>

#include "types/system_defined.h"
#include "core/BaseRobotConfig.h"
#include "types/joystickTypes.h"

class MujocoManager {
public:
  MujocoManager(std::shared_ptr<const BaseRobotConfig> cfg,
                jointCMD* jointCMDPtr,
                robotStatus* robotStatusPtr);
  ~MujocoManager();

  void initWorld();
  void moveToDefaultPose();
  void initState();
  void launchServer();
  void renderLoop();
  void run();
  void integrate();
  void updateRobotState();
  void stop() { running_ = false; glfwSetWindowShouldClose(window_, GLFW_TRUE); }
  void setUserInputPtr(char* key, JoystickData* joy) { joyPtr_ = joy; keyPtr_ = key; }

private:
  std::shared_ptr<const BaseRobotConfig> cfg_;

  std::string robotName_;
  float control_dt_ = 2e-3;
  float simulation_dt_ = 5e-4;

  bool running_ = true;
  bool isStatesReady = false;

  int gcDim_ = 0, gvDim_ = 0;
  int jointDim_ = 0;

  robotStatus* robotStatusPtr_ = nullptr;
  jointCMD* jointCMDPtr_ = nullptr;

  JoystickData* joyPtr_ = nullptr;
  char* keyPtr_ = nullptr;

  std::mutex state_lock_;
  std::mutex action_lock_;

  Eigen::VectorXd gc_, gv_;
  Eigen::VectorXf pTarget, vTarget;
  Eigen::VectorXf tauCmd;
  Eigen::VectorXf jointPGain, jointDGain;

  // MuJoCo core members
  mjModel* mj_model_ = nullptr;
  mjData* mj_data_ = nullptr;
  GLFWwindow* window_ = nullptr;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
};