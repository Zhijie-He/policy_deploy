#include <filesystem>
#include <csignal>
#include "controller/NeuralController.h"
#include "state_machine/StateMachine.h"
#include "hardware/listener.h"
#include "sim2/simulator/g1_sim2mujoco_env.h"
#include "utility/tools.h"
#include "utility/cxxopts.hpp"

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

std::shared_ptr<BaseRobotConfig> cfg = nullptr;
std::shared_ptr<Listener> listener = nullptr;
std::shared_ptr<StateMachine> state_machine = nullptr;
std::shared_ptr<G1Sim2MujocoEnv> env = nullptr; 

void close_all_threads(int signum) {
  FRC_INFO("Interrupted with SIGINT: " << signum << "\n");
  if (env != nullptr) env->stop();
  if (listener != nullptr) listener->stop();
  if (state_machine != nullptr) state_machine->stop();
  std::exit(0);
}

int main(int argc, char** argv) {
  std::string exec_name = std::filesystem::path(argv[0]).filename().string();
  std::string config_name;
  bool headless = false;
  torch::Device torchDevice = torch::kCPU;

  try {
    cxxopts::Options options("deploy_mujoco", "Run Mujoco-based simulation for Human Legged Robot");
    options.add_options()
        ("c,config", "Config name (e.g., g1_unitree, g1_eman, h1, h1_2)", cxxopts::value<std::string>())
        ("headless", "Run in headless mode (no GUI)", cxxopts::value<bool>()->default_value("false"))
        ("d,device", "Device to use: cpu or cuda", cxxopts::value<std::string>()->default_value("cpu"))
        ("h,help", "Show help");

    auto result = options.parse(argc, argv);
    if (result.count("help") || !result.count("config")) {
        FRC_INFO("\n" << options.help());
        return 0;
    }
    
    config_name = result["config"].as<std::string>();
    headless = result["headless"].as<bool>();

    std::string device_str = result["device"].as<std::string>();
    std::transform(device_str.begin(), device_str.end(), device_str.begin(), ::tolower);

    if (device_str != "cpu" && device_str != "cuda") {
      FRC_ERROR("[Argument Error] Invalid device: " << device_str << ", must be 'cpu' or 'cuda'");
      return 1;
    }

    if (device_str == "cuda") {
      torchDevice = tools::getDefaultDevice();
      if (torchDevice.type() != torch::kCUDA) {
          FRC_WARN("CUDA requested, but not available on this machine. Falling back to CPU.");
          device_str = "cpu";
      }
    }

  } catch (const std::exception& e) {
    FRC_ERROR("[Argument Parsing Error] " << e.what());
    return 1;
  }

  signal(SIGINT, close_all_threads);
  
  cfg = tools::loadConfig(config_name);
  listener = std::make_shared<Listener>();
  state_machine = std::make_shared<StateMachine>(cfg, config_name, torchDevice);
  env = std::make_shared<G1Sim2MujocoEnv>(cfg,
                                          state_machine->getJointCMDBufferPtr(),
                                          state_machine->getRobotStatusBufferPtr());
  env->setHeadless(headless); 
  env->setUserInputPtr(listener, listener->getKeyInputPtr(), nullptr);
  state_machine->setInputPtr(listener->getKeyInputPtr(), nullptr);
  
  std::thread keyboard_thread(&Listener::listenKeyboard, listener); 
  std::thread ctrl_thread(&StateMachine::run, state_machine);
  
  // Move to the default position
  env->moveToDefaultPos();
  
  env->run();
  
  close_all_threads(404);
  ctrl_thread.join();
  keyboard_thread.join();
  return 0;
}
