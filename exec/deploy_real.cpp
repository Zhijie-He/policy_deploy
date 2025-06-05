#include <yaml-cpp/yaml.h>
#include <torch/torch.h>
#include <iostream>
#include <cmath>
#include <memory>
#include <filesystem>
#include <csignal>
#include "controller/NeuralController.h"
#include "controller/PolicyWrapper.h"
#include "controller/EmanPolicyWrapper.h"
#include "state_machine/StateMachine.h"
#include "core/BaseRobotConfig.h"
#include "core/EmanRobotConfig.h"
#include "core/RobotConfig.h"
#include "hardware/listener.h"
#include "real/G1Sim2RealEnv.h"  
#include "utility/MathUtilities.h"
#include "controller/ResetController.h"

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

class NeuralRunner: public StateMachine {
public:
  explicit NeuralRunner(std::shared_ptr<const BaseRobotConfig> cfg, const std::string& robot_name) : StateMachine(cfg) {
    FRC_INFO("[NeuralRunner.Const] NeuralRunner init");
   
    if (robot_name == "g1_unitree") {
      _neuralCtrl = std::make_unique<PolicyWrapper>(cfg);
    } else if (robot_name == "g1_eman") {
      _neuralCtrl = std::make_unique<EmanPolicyWrapper>(cfg);
    } else {
      throw std::runtime_error("Unsupported robot: " + robot_name);
    }
  }

  void step() override {
    parseRobotData();
    updateCommands();
    robotAction = _neuralCtrl->getControlAction(robotData);
    packJointAction();
    if (*_keyState != '\0') *_keyState = '\0';
  }

  void stop() override {
    _isRunning = false;
  }

private:
  std::unique_ptr<NeuralController> _neuralCtrl;
};

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
  // 参数数量校验
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

  // 构造 config 路径
  std::string config_path = std::string(PROJECT_SOURCE_DIR) + "/config/" + config_name + ".yaml";
  // 加载对应的 robot config 类型
  if (config_name == "g1_unitree") {
      cfg = std::make_shared<RobotConfig>(config_path);
  } else if (config_name == "g1_eman") {
      cfg = std::make_shared<EmanRobotConfig>(config_path);
  } else {
      throw std::runtime_error("Unsupported robot config: " + config_name);
  }

  listener = std::make_shared<Listener>();
  ctrl = std::make_shared<NeuralRunner>(cfg, config_name);
  sim = std::make_shared<G1Sim2RealEnv>(
            cfg,
            net_interface,
            ctrl->getJointCMDPtr(),
            ctrl->getRobotStatusPtr());
  sim->setUserInputPtr(listener->getKeyInputPtr(), nullptr);
  ctrl->setInputPtr(listener->getKeyInputPtr(), nullptr);
  
  return 0;
}