#pragma once

#include <atomic>
#include <cstdint>
#include "hardware/joystick.h"

// 键盘与遥控器监听器
class Listener {
 public:
  Listener() = default;
  ~Listener();

  void startKeyboardListener();
  void startJoystickListener(int id = 0);
  void stop();
  void join();

  bool isRunning() const { return isRunning_.load(); }
  char getKeyboardInput() const { return key_input_.load(); }
  void clearKeyboardInput() { key_input_.store('\0'); }
  
  Joystick getJoystickState() const {
    return gamepad_.getJoystick();
  }
  JoystickService gamepad_;
  
private:
  std::atomic<bool> isRunning_{false};
  std::atomic<char> key_input_ = {'\0'};
  
  std::thread keyboardThread_;

};

