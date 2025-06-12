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
#include "types/CustomTypes.h"

class G1Sim2MujocoEnv : public BaseEnv {
public:
  G1Sim2MujocoEnv(std::shared_ptr<const BaseRobotConfig> cfg,
                std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr,
                std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr);

  G1Sim2MujocoEnv(const std::string& net_interface,
            std::shared_ptr<const BaseRobotConfig> cfg,
            const std::string& mode,
            const std::string& track,
            const std::vector<std::string>& track_list,
            std::shared_ptr<CustomTypes::MocapConfig> mocap_cfg,  
            std::shared_ptr<CustomTypes::VlaConfig> vla_cfg,
            std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr,
            std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr);   

  ~G1Sim2MujocoEnv();

  void initWorld();
  void initState();
  void launchServer();
  void renderLoop() override;
  void run() override;
  void integrate() override;
  void updateRobotState();
  void stop() override;
  void moveToDefaultPos() override;

private:
  double lastx_ = 0, lasty_ = 0;
  std::string robotName_;
  float simulation_dt_ = 5e-4;
  bool isStatesReady = false;
  
  Eigen::VectorXf tauCmd;
  std::atomic<bool> isFirstActionReceived = false;

  // MuJoCo core members
  mjModel* mj_model_ = nullptr;
  mjData* mj_data_ = nullptr;
  GLFWwindow* window_ = nullptr;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
};