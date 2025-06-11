#include <yaml-cpp/yaml.h>
#include <iostream>
#include <memory>
#include <filesystem>
#include <csignal>
#include "state_machine/NeuralRunner.h"
#include "core/EmanRobotConfig.h"
#include "core/RobotConfig.h"
#include "hardware/listener.h"
#include "sim2/g1_sim2mujoco_env.h"

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

std::shared_ptr<BaseRobotConfig> cfg = nullptr;
std::shared_ptr<Listener> listener = nullptr;
std::shared_ptr<NeuralRunner> ctrl = nullptr;
std::shared_ptr<BaseEnv> env = nullptr; 

void close_all_threads(int signum) {
  FRC_INFO("Interrupted with SIGINT: " << signum << "\n");
  if (env != nullptr) env->stop();
  if (listener != nullptr) listener->stop();
  if (ctrl != nullptr) ctrl->stop();
  std::exit(0);
}

std::shared_ptr<BaseRobotConfig> loadConfig(const std::string& config_name) {
  const std::string config_path = std::string(PROJECT_SOURCE_DIR) + "/config/" + config_name + ".yaml";
  if (config_name == "g1_unitree")
    return std::make_shared<RobotConfig>(config_path);
  else if (config_name == "g1_eman")
    return std::make_shared<EmanRobotConfig>(config_path);
  else
    throw std::runtime_error("Unsupported robot config: " + config_name);
}

int main(int argc, char** argv) {
  std::string exec_name = std::filesystem::path(argv[0]).filename().string();
  if (argc < 2) {
    FRC_ERROR("Usage error: Please provide a config name, e.g., ./" << exec_name << " g1_unitree");
    return -1;
  }

  signal(SIGINT, close_all_threads);

  std::string config_name = argv[1];
  cfg = loadConfig(config_name);
  listener = std::make_shared<Listener>();
  ctrl = std::make_shared<NeuralRunner>(cfg, config_name);

  std::string mode = "sim2mujoco", track = "cmd";
  if (mode == "sim2mujoco") {
    env = std::make_shared<G1Sim2MujocoEnv>(
      cfg,
      ctrl->getJointCMDBufferPtr(),
      ctrl->getRobotStatusBufferPtr());
  } else {
    FRC_ERROR("Unsupported mode: " << mode);
    return -1;
  }

  env->setUserInputPtr(listener->getKeyInputPtr(), nullptr);
  ctrl->setInputPtr(listener->getKeyInputPtr(), nullptr);
    
  std::thread keyboard_thread([&]() { listener->listenKeyboard(); });
  std::thread ctrl_thread([&]() { ctrl->run(); });
  std::thread comm_thread([&]() { env->run(); });
  std::thread integrate_thread([&]() { env->integrate(); });

  env->renderLoop();

  ctrl->stop();
  listener->stop();
  ctrl_thread.join();
  keyboard_thread.join();
  comm_thread.join();
  integrate_thread.join();

  return 0;
}
