#include <yaml-cpp/yaml.h>
#include <torch/torch.h>
#include <iostream>
#include <cmath>
#include <memory>
#include <filesystem>
#include <csignal>
#include "controller/NeuralController.h"
#include "state_machine/NeuralRunner.h"
#include "core/EmanRobotConfig.h"
#include "core/RobotConfig.h"
#include "hardware/listener.h"
#include "real/G1Sim2RealEnv.h"  
#include "utility/MathUtilities.h"
#include "types/CustomTypes.h"

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

std::shared_ptr<BaseRobotConfig> cfg = nullptr;
std::shared_ptr<Listener> listener = nullptr;
std::shared_ptr<NeuralRunner> ctrl = nullptr;
std::shared_ptr<G1Sim2RealEnv> sim = nullptr; 

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
      FRC_ERROR("Usage error: Please provide a config name, e.g., ./" << exec_name << " g1_unitree [net_interface]");
      return -1;
  }
  std::string config_name = argv[1];
  std::string net_interface = (argc >= 3) ? argv[2] : "";

  // 判断是否真机配置
  bool is_real_robot = true;
  if (net_interface.empty() && is_real_robot) {
      FRC_ERROR("Missing network interface for real robot config: " << config_name);
      FRC_ERROR("Usage: ./" << exec_name << " " << config_name << " [net_interface]");
      return -1;
  }
  signal(SIGINT, close_all_threads);

  std::string config_path = std::string(PROJECT_SOURCE_DIR) + "/config/" + config_name + ".yaml";
  if (config_name == "g1_unitree") {
    cfg = std::make_shared<RobotConfig>(config_path);
  } else if (config_name == "g1_eman") {
    cfg = std::make_shared<EmanRobotConfig>(config_path);
  } else {
    FRC_ERROR("Unsupported robot config: " << config_name);
    throw std::runtime_error("Unsupported robot config: " + config_name);
  }

  listener = std::make_shared<Listener>();
  ctrl = std::make_shared<NeuralRunner>(cfg, config_name);

  // mode: sim2mujoco, sim2real
  // track: cmd, fpos, mocap, vla
  std::string mode = "sim2real", track = "cmd";
  std::vector<std::string> track_list = {"cmd"};
  std::shared_ptr<CustomTypes::MocapConfig> mocap_cfg = nullptr;
  std::shared_ptr<CustomTypes::VlaConfig> vla_cfg = nullptr;

  // std::vector<std::string> track_list = {"cmd", "mocap"};
  // auto mocap_cfg = std::make_shared<CustomTypes::MocapConfig>("192.168.123.111", 7003, 30);

  // std::vector<std::string> track_list = {"cmd", "vla"};
  // auto vla_cfg = std::make_shared<CustomTypes::VlaConfig>("example_data.pkl");

  sim = std::make_shared<G1Sim2RealEnv>(
            cfg,
            net_interface,
            ctrl->getJointCMDBufferPtr(),
            ctrl->getRobotStatusBufferPtr(),
            mode,
            track,
            track_list,
            mocap_cfg,
            vla_cfg);
            
  sim->setUserInputPtr(listener->getKeyInputPtr(), nullptr, listener);
  ctrl->setInputPtr(listener->getKeyInputPtr(), nullptr);
  
  std::thread simuRobot_thread(&G1Sim2RealEnv::simulateRobot, sim);
  std::thread keyboard_thread(&Listener::listenKeyboard, listener);
  std::thread ctrl_thread(&StateMachine::run, ctrl);

  // Enter the zero torque state, press the start key to continue executing
  if(mode=="sim2real") {
    sim->zeroTorqueState();
  }
  
  // Move to the default position
  sim->moveToDefaultPos();
  
  // Enter the default position state, press the A key to continue executing
  if(mode=="sim2real") {
    sim->defaultPosState();
  }

  std::thread comm_thread(&G1Sim2RealEnv::run, sim);
  
  simuRobot_thread.join();
  comm_thread.join();
  return 0;
}
