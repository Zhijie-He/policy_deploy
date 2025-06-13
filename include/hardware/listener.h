#pragma once

#include <atomic>
#include <cstdint>
#include "gamepad.hpp"

// 键盘与遥控器监听器
class Listener {
 public:
  Listener() = default;
  void listenKeyboard();
  void stop() { isRunning_.store(false); }
  bool isRunning() const { return isRunning_.load(); }
  char* getKeyInputPtr() { return &key_input_; }
  
  // unitree official
  unitree::common::Gamepad gamepad_;
  unitree::common::REMOTE_DATA_RX rx_;

 private:
  std::atomic<bool> isRunning_{true};
  char key_input_ = '\0';
};

