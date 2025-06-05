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
#include "real/G1Manager.h"  
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
std::shared_ptr<G1Manager> sim = nullptr; 

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
  signal(SIGINT, close_all_threads);

  std::string config_path = std::string(PROJECT_SOURCE_DIR) + "/config/" + argv[1] + ".yaml";
  if (argv[1] == std::string("g1_unitree")) {
      cfg = std::make_shared<RobotConfig>(config_path);
  } else if (argv[1] == std::string("g1_eman")) {
      cfg = std::make_shared<EmanRobotConfig>(config_path);
  } else {
      throw std::runtime_error("Unsupported robot config: " + std::string(argv[1]));
  }

  listener = std::make_shared<Listener>();
  ctrl = std::make_shared<NeuralRunner>(cfg, argv[1]);
  sim = std::make_shared<G1Manager>(
            cfg,
            ctrl->getJointCMDPtr(),
            ctrl->getRobotStatusPtr());

  sim->setUserInputPtr(listener->getKeyInputPtr(), nullptr);
  ctrl->setInputPtr(listener->getKeyInputPtr(), nullptr);

  std::thread keyboard_thread(&Listener::listenKeyboard, listener);
  std::thread ctrl_thread(&StateMachine::run, ctrl);
  std::thread comm_thread(&G1Manager::run, sim);
  std::thread integrate_thread(&G1Manager::integrate, sim);

  sim->renderLoop();

  // 关闭线程（在 sim.run() 退出后执行）
  ctrl->stop();
  listener->stop();
  ctrl_thread.join();
  keyboard_thread.join();
  comm_thread.join();
  integrate_thread.join();

  return 0;
}