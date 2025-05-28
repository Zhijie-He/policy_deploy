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
#include "simulator/RaisimManager.h"
#include "utility/MathUtilities.h"
#include "controller/ResetController.h"

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

class NeuralRunner: public StateMachine{
  public:
    explicit NeuralRunner(std::shared_ptr<const RobotConfig> cfg) : StateMachine(cfg) { 
      FRC_INFO("NeuralRunner init");

      // 1. 打印调试信息（站立姿态、KP）
      FRC_INFO("[FSM.init] KP_home = " << cfg->homingKp.transpose());
      FRC_INFO("[FSM.init] KD_home = " << cfg->homingKd.transpose());
      FRC_INFO("[FSM.init] Pos_home = " << cfg->homingPos.transpose());

      // 2. 创建归位控制器（让机器人先站好）
      _homingCtrl = std::make_unique<ResetController>(cfg->homingPos, cfg->homing_timesteps, cfg->getPolicyDt());
      _homingCtrl->setPDGains(cfg->homingKp, cfg->homingKd);

      // 3. 创建神经网络控制器（正式运行） 
      _neuralCtrl = std::make_unique<PolicyWrapper>(cfg);
    }
    
    void step() override{
      // ➡️ 从传感器结构体中读取当前机器人状态，如位置、速度、姿态角、IMU 数据等（见下文解释）。 1️⃣ 读取当前机器人状态
      parseRobotStates();

      // 2️⃣ 判断是否切换到策略控制模式
      if (_homingCtrl->isComplete() && !_isPolicyActive && _keyState && *_keyState == ' ') {
        // ✅ 当归位控制完成（homing），且尚未启用策略控制（policy），且按下了空格键 ' ' 时，就切换到策略控制模式，并初始化目标状态。
        _isPolicyActive = true;
        yawTarg = robotState.baseRpy[2]; // ➡️ 记录当前的 yaw 角（偏航角）作为目标方向，可以用于之后的朝向保持或旋转参考。
        robotState.targetVelocity.setZero(); // ➡️ 将目标速度设为零，表示一开始站着不动，等待用户按方向键（比如 'w'）来设定速度。
      }

      // 3️⃣ 更新目标速度/方向
      updateCommands();

      // 4️⃣ 控制方式分支：策略 or homing 如果策略激活（按了空格），就运行神经网络控制器 getControlAction()；
      if (_isPolicyActive) {
        robotAction = _neuralCtrl->getControlAction(robotState);
      }
      else{
        robotAction = _homingCtrl->getControlAction(robotState);
      }

      // 5️⃣ 打包控制动作，准备输出
      packRobotAction();
      // 6️⃣ 清空按键输入，避免多次触发 避免按一次 'w' 被连续多帧处理
      if (*_keyState != '\0') *_keyState='\0';
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
std::shared_ptr<RaisimManager> sim = nullptr;

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
  sim = std::make_shared<RaisimManager>( // 这段代码的含义是：创建一个 RaisimManager 实例，并传入配置与三个传感器/控制数据指针，赋值给 sim。
                                  cfg,
                                  ctrl->getGyroPtr(),          // ← 读取 StateMachine 内部的 IMU 指针
                                  ctrl->getMotorStatePtr(),    // ← 当前电机状态
                                  ctrl->getMotorTargetPtr()    // ← 输出目标状态
                                  );  
  sim->setUserInputPtr(listener->getKeyInputPtr(), nullptr);
  ctrl->setInputPtr(listener->getKeyInputPtr(), nullptr);

  std::cin.get(); 

  std::thread keyboard_thread(&Listener::listenKeyboard, listener);
  std::thread comm_thread(&RaisimManager::run, sim);
  std::thread ctrl_thread(&StateMachine::run, ctrl);
  std::thread world_thread(&RaisimManager::integrate, sim);

  world_thread.join(); 
  return 0;
}
