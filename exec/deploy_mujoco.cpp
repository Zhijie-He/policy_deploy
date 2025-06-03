#include <yaml-cpp/yaml.h>
#include <torch/torch.h>
#include <iostream>
#include <cmath>
#include <memory>
#include <filesystem>
#include <csignal>
#include "controller/PolicyWrapper.h"
#include "state_machine/StateMachine.h"
#include "core/RobotConfig.h"
#include "hardware/listener.h"
#include "simulator/MujocoManager.h"  
#include "utility/MathUtilities.h"
#include "controller/ResetController.h"

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

class NeuralRunner: public StateMachine {
public:
  explicit NeuralRunner(std::shared_ptr<const RobotConfig> cfg) : StateMachine(cfg) {
    FRC_INFO("NeuralRunner init");

    FRC_INFO("[FSM.init] KP_home = " << cfg->homingKp.transpose());
    FRC_INFO("[FSM.init] KD_home = " << cfg->homingKd.transpose());
    FRC_INFO("[FSM.init] Pos_home = " << cfg->homingPos.transpose());

    _homingCtrl = std::make_unique<ResetController>(cfg);
    _neuralCtrl = std::make_unique<PolicyWrapper>(cfg);
  }

  void step() override {
    parseRobotStates();

    if (_homingCtrl->isComplete() && !_isPolicyActive && _keyState && *_keyState == ' ') {
      _isPolicyActive = true;
      yawTarg = robotState.baseRpy[2];
      robotState.targetVelocity.setZero();
    }

    updateCommands();

    if (_isPolicyActive)
      robotAction = _neuralCtrl->getControlAction(robotState);
    else
      robotAction = _homingCtrl->getControlAction(robotState);

    packRobotAction();
    if (*_keyState != '\0') *_keyState = '\0';
  }

  void stop() override {
    _isRunning = false;
  }

private:
  bool _isPolicyActive = true;
  std::unique_ptr<ResetController> _homingCtrl;
  std::unique_ptr<PolicyWrapper> _neuralCtrl;
};

std::shared_ptr<RobotConfig> cfg = nullptr;
std::shared_ptr<Listener> listener = nullptr;
std::shared_ptr<NeuralRunner> ctrl = nullptr;
std::shared_ptr<MujocoManager> sim = nullptr;  // ⬅️ 替换 RaisimManager

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
    FRC_ERROR("Usage error: Please provide a config name, e.g., ./" << exec_name << " g1");
    return -1;
  }
  signal(SIGINT, close_all_threads);

  std::string config_path = std::string(PROJECT_SOURCE_DIR) + "/config/" + argv[1] + ".yaml";
  cfg = std::make_shared<RobotConfig>(config_path);
  listener = std::make_shared<Listener>();
  ctrl = std::make_shared<NeuralRunner>(cfg);
  sim = std::make_shared<MujocoManager>(
            cfg,
            ctrl->getGyroPtr(),
            ctrl->getMotorStatePtr(),
            ctrl->getMotorTargetPtr());

  sim->setUserInputPtr(listener->getKeyInputPtr(), nullptr);
  ctrl->setInputPtr(listener->getKeyInputPtr(), nullptr);

  // std::cin.get(); 

  // 仅控制线程使用 std::thread，主线程负责 GUI（OpenGL）
  std::thread keyboard_thread(&Listener::listenKeyboard, listener);
  std::thread ctrl_thread(&StateMachine::run, ctrl);

  // 主线程执行 Mujoco 渲染（必须）
  sim->run();

  // 关闭线程（在 sim.run() 退出后执行）
  ctrl->stop();
  listener->stop();
  ctrl_thread.join();
  keyboard_thread.join();

  return 0;
}