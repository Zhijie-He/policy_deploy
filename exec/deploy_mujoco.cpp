#include <yaml-cpp/yaml.h>
#include <torch/torch.h>
#include <iostream>
#include <cmath>
#include <memory>
#include <filesystem>
#include <csignal>
#include <mujoco/mujoco.h>
#include "controller/NeuralController.h"
#include "state_machine/NeuralRunner.h"
#include "core/EmanRobotConfig.h"
#include "core/RobotConfig.h"
#include "hardware/listener.h"
#include "simulator/MujocoManager.h"  
#include "utility/MathUtilities.h"

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

std::shared_ptr<BaseRobotConfig> cfg = nullptr;
std::shared_ptr<Listener> listener = nullptr;
std::shared_ptr<NeuralRunner> ctrl = nullptr;
std::shared_ptr<MujocoManager> sim = nullptr; 

void close_all_threads(int signum) {
  FRC_INFO("Interrupted with SIGINT: " << signum << "\n");
  if (sim != nullptr) sim->stop();
  if (listener != nullptr) listener->stop();
  if (ctrl != nullptr) ctrl->stop();
  std::exit(0);
}

int main(int argc, char** argv) {
  std::string exec_name = std::filesystem::path(argv[0]).filename().string();
  if (argc < 2) {
    FRC_ERROR("Usage error: Please provide a config name, e.g., ./" << exec_name << " g1_unitree");
    return -1;
  }

  // Mujoco版本检查
  if (mjVERSION_HEADER != mj_version()) {
    FRC_ERROR("MuJoCo header and library version mismatch!");
    return -1;
  }
  FRC_INFO("MuJoCo Version: " << mj_version());

  signal(SIGINT, close_all_threads);

  std::string config_name = argv[1];
  std::string config_path = std::string(PROJECT_SOURCE_DIR) + "/config/" + config_name + ".yaml";
  if (config_name == "g1_unitree") {
      cfg = std::make_shared<RobotConfig>(config_path);
  } else if (config_name == "g1_eman") {
      cfg = std::make_shared<EmanRobotConfig>(config_path);
  } else {
      throw std::runtime_error("Unsupported robot config: " + std::string(config_name));
  }

  listener = std::make_shared<Listener>();
  ctrl = std::make_shared<NeuralRunner>(cfg, config_name);
  sim = std::make_shared<MujocoManager>(
            cfg,
            ctrl->getJointCMDBufferPtr(),
            ctrl->getRobotStatusBufferPtr());

  sim->setUserInputPtr(listener->getKeyInputPtr(), nullptr);
  ctrl->setInputPtr(listener->getKeyInputPtr(), nullptr);
    
  std::thread keyboard_thread(&Listener::listenKeyboard, listener);
  std::thread ctrl_thread(&StateMachine::run, ctrl);
  std::thread comm_thread(&MujocoManager::run, sim);
  std::thread integrate_thread(&MujocoManager::integrate, sim);

  sim->renderLoop();

  ctrl->stop();
  listener->stop();
  ctrl_thread.join();
  keyboard_thread.join();
  comm_thread.join();
  integrate_thread.join();

  return 0;
}
