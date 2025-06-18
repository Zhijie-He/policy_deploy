#pragma once

#include <yaml-cpp/yaml.h>
#include <utility>
#include <mutex>
#include "types/motor_data.h"
#include "types/joystickTypes.h"
#include "types/imu_types.h"
#include "types/CustomTypes.h"


class StateMachine{
public:
  StateMachine(const YAML::Node& cfg);
  virtual ~StateMachine() = default;

  void run() {};
  virtual void step() {}
  virtual void reset() {}
  virtual void stop() {};

  void updateAction();

  void setJoystickPtr(const std::shared_ptr<JoystickData>& joy) {
    if (joy == nullptr) return;
    _joystickBuf = joy;
  }

  std::shared_ptr<JoystickData> getJoystickPtr() {
    return _joystickBuf;
  }


  std::shared_ptr<GyroData> getGyroPtr() {
    return _gyroStateBuf;
  }
  void setGyroPtr(const std::shared_ptr<GyroData>& state) {
    if (state == nullptr) return;
    _gyroStateBuf = state;
  }

  std::shared_ptr<motor_state_data> getMotorStatePtr() {
    return _jointStateBuf;
  }

  void setMotorStatePtr(const std::shared_ptr<motor_state_data>& state) {
    if (state == nullptr) return;
    _jointStateBuf = state;
  }

  std::shared_ptr<motor_cmd_data> getMotorTargetPtr() {
    return _jointCmdBuf;
  }

  void setMotorTargetPtr(const std::shared_ptr<motor_cmd_data>& cmd) {
    if (cmd == nullptr) return;
    _jointCmdBuf = cmd;
  }

  bool isRunning() {return _isRunning;}

protected:

  bool isFiltersEnable_ = false;

  bool _isStateReady = false;
  bool _isRunning = true;
  int _jointNum = 14;
  int _ecMotorNum = 14;
  int _armMotorNum = 8;
  float _policyDt = 0.001;
  std::chrono::milliseconds _policyDtMs = std::chrono::milliseconds(static_cast<int>(_policyDt * 1000 + 0.001));

  std::shared_ptr<JoystickData> _joystickBuf = nullptr;
  std::shared_ptr<motor_cmd_data> _jointCmdBuf = nullptr;
  std::shared_ptr<motor_state_data> _jointStateBuf = nullptr;
  std::shared_ptr<GyroData> _gyroStateBuf = nullptr;
  std::shared_ptr<JoystickData> _jsStates = nullptr;
  char* _keyState = nullptr;
  CustomTypes::State robotState;
  CustomTypes::Action robotAction;

  float yawTarg = 0;

  std::mutex _mutexGyro;
  std::mutex _mutexCmd;
  std::mutex _mutexJoint;
  std::mutex _mutexJoy;
};

