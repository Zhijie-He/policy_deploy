
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <memory>
#include <filesystem>
#include <csignal>
#include "state_machine/StateMachine.h"
#include "config/EmanRobotConfig.h"
#include "config/UnitreeRobotConfig.h"
#include "hardware/listener.h"
#include "sim2/real/g1_sim2real_env.h"
#include "sim2/simulator/g1_sim2mujoco_env.h"
#include "types/CustomTypes.h"

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

std::shared_ptr<BaseRobotConfig> cfg = nullptr;
std::shared_ptr<Listener> listener = nullptr;
std::shared_ptr<StateMachine> ctrl = nullptr;
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
    return std::make_shared<UnitreeRobotConfig>(config_path);
  else if (config_name == "g1_eman")
    return std::make_shared<EmanRobotConfig>(config_path);
  else
    throw std::runtime_error("Unsupported robot config: " + config_name);
}

int main(int argc, char** argv) {
  std::string exec_name = std::filesystem::path(argv[0]).filename().string();
  // 参数检查
  if (argc < 3) {
      FRC_ERROR("Usage: " << exec_name << " <mode> <config_name> [net]");
      FRC_ERROR("Example: " << exec_name << " sim2mujoco g1_eman");
      FRC_ERROR("         " << exec_name << " sim2real g1_eman net");
      return -1;
  }

  std::string mode = argv[1];        // sim2mujoco or sim2real
  std::string config_name = argv[2]; // config name like g1_eman
  std::string net = (argc >= 4) ? argv[3] : "";

  // 模式合法性检查
  if (mode != "sim2mujoco" && mode != "sim2real") {
    FRC_ERROR("Invalid mode: " << mode);
    FRC_ERROR("Available modes: sim2mujoco | sim2real");
    return -1;
  }

  // 配置合法性检查
  if (config_name != "g1_unitree" && config_name != "g1_eman") {
    FRC_ERROR("Invalid config name: " << config_name);
    FRC_ERROR("Available config names: g1_unitree | g1_eman");
    return -1;
  }

  // sim2real 限制：必须带 net 参数
  if (mode == "sim2real" && net.empty()) {
    FRC_ERROR("Missing <net> argument for mode 'sim2real'.");
    FRC_ERROR("Usage: " << exec_name << " sim2real g1_eman net");
    return -1;
  }

  // sim2real 限制：只支持 g1_eman，不支持 g1_unitree
  if (mode == "sim2real" && config_name == "g1_unitree") {
    FRC_ERROR("Mode 'sim2real' is not supported with config 'g1_unitree'.");
    FRC_ERROR("Only 'g1_eman' is supported in sim2real mode.");
    return -1;
  }

  signal(SIGINT, close_all_threads);

  cfg = loadConfig(config_name);
  listener = std::make_shared<Listener>();
  ctrl = std::make_shared<StateMachine>(cfg, config_name);

  // mode: sim2mujoco, sim2real
  // track: cmd, fpos, mocap, vla
  std::string track = "cmd";
  std::vector<std::string> track_list = {"cmd"};
  std::shared_ptr<CustomTypes::MocapConfig> mocap_cfg = nullptr;
  std::shared_ptr<CustomTypes::VlaConfig> vla_cfg = nullptr;

  // std::vector<std::string> track_list = {"cmd", "mocap"};
  // auto mocap_cfg = std::make_shared<CustomTypes::MocapConfig>("192.168.123.111", 7003, 30);

  // std::vector<std::string> track_list = {"cmd", "vla"};
  // auto vla_cfg = std::make_shared<CustomTypes::VlaConfig>("example_data.pkl");
  
  if (mode == "sim2mujoco") {
    env = std::make_shared<G1Sim2MujocoEnv>(net,
                                            cfg,
                                            mode,
                                            track,
                                            track_list,
                                            mocap_cfg,
                                            vla_cfg,
                                            ctrl->getJointCMDBufferPtr(),
                                            ctrl->getRobotStatusBufferPtr());
  } else if(mode == "sim2real" && config_name == "g1_eman") {
    env = std::make_shared<G1Sim2RealEnv>(net,
                                          cfg,
                                          mode,
                                          track,
                                          track_list,
                                          mocap_cfg,
                                          vla_cfg,
                                          ctrl->getJointCMDBufferPtr(),
                                          ctrl->getRobotStatusBufferPtr());
  } else {
    FRC_ERROR("Unsupported mode: " << mode << " and config: " << config_name);
    return -1;
  }

  env->setUserInputPtr(listener, listener->getKeyInputPtr(), nullptr);
  ctrl->setInputPtr(listener->getKeyInputPtr(), nullptr);
    
  std::thread keyboard_thread([&]() { listener->listenKeyboard(); }); // start listener
  std::thread ctrl_thread([&]() { ctrl->run(); }); // start policy
  std::thread simuRobot_thread;
  if(mode=="sim2real"){
    simuRobot_thread = std::thread([&]() { env->simulateRobot(); }); // simulate G1 robotstatus
  }

  // Enter the zero torque state, press the start key to continue executing
  if(mode=="sim2real") {
    env->zeroTorqueState();
  }
  
  // Move to the default position
  env->moveToDefaultPos();
  
  // Enter the default position state, press the A key to continue executing
  if(mode=="sim2real") {
    env->defaultPosState();
  }

  if(mode == "sim2mujoco"){
    std::thread integrate_thread([&]() { env->integrate(); });
    std::thread comm_thread([&]() { env->run(); });

    env->renderLoop();

    ctrl->stop();
    listener->stop();

    ctrl_thread.join();
    keyboard_thread.join();
    comm_thread.join();
    integrate_thread.join();
  }else if(mode=="sim2real"){
    env->run();
    simuRobot_thread.join();
  }
  return 0;
}
