#pragma once

#include <memory>
#include "state_machine/StateMachine.h"
#include "controller/PolicyWrapper.h"
#include "controller/EmanPolicyWrapper.h"
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