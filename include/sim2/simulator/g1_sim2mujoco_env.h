// ===== include/sim2/simulator/g1_sim2mujoco_env.h =====
#pragma once

#include <yaml-cpp/yaml.h>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <memory>
#include <mutex>
#include <Eigen/Core>
#include <vector>
#include "types/system_defined.h"
#include "sim2/base_env.h"
#include "state_machine/StateMachine.h"

class G1Sim2MujocoEnv : public BaseEnv {
public:
  G1Sim2MujocoEnv(std::shared_ptr<const BaseRobotConfig> cfg,
                std::shared_ptr<StateMachine> state_machine);

  G1Sim2MujocoEnv(std::shared_ptr<const BaseRobotConfig> cfg,
                std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr,
                std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr);

  ~G1Sim2MujocoEnv();

  void initWorld();
  void launchServer();
  void setHeadless(bool headless) override { headless_ = headless; }
  void run() override;
  void step();
  void integrate();
  void updateRobotState();
  void stop() override;
  void moveToDefaultPos() override;

private:
  std::shared_ptr<StateMachine> state_machine_ = nullptr;
  double lastx_ = 0, lasty_ = 0;
  std::string robotName_;
  float simulation_dt_ = 5e-4;

  // MuJoCo core members
  bool headless_ = true; 
  mjModel* mj_model_ = nullptr;
  mjData* mj_data_ = nullptr;
  GLFWwindow* window_ = nullptr;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
};