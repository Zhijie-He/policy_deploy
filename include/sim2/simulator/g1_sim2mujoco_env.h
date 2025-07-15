// ===== include/sim2/simulator/g1_sim2mujoco_env.h =====
#pragma once

#include <mutex>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <GLFW/glfw3.h>
#include <yaml-cpp/yaml.h>
#include <mujoco/mujoco.h>
#include "sim2/base_env.h"
#include "types/system_defined.h"

class G1Sim2MujocoEnv : public BaseEnv {
public:
  G1Sim2MujocoEnv(std::shared_ptr<const BaseRobotConfig> cfg,
                  const std::string& hands_type,
                  std::shared_ptr<StateMachine> state_machine);

  ~G1Sim2MujocoEnv();

  void initWorld() override;
  void launchServer();
  void setHeadless(bool headless) override { headless_ = headless; }
  void run() override;
  void step();
  void integrate();
  void updateRobotState();
  void stop() override;
  void moveToDefaultPos() override;
  
  bool isRunning() const override; 
  void applyAction(const jointCMD& cmd) override;  

private:
  double lastx_ = 0, lasty_ = 0;
  float simulation_dt_ = 5e-4;
  std::thread step_thread_;

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