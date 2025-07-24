#pragma once

#include <atomic>
#include <cstdint>

// 键盘与遥控器监听器
class Listener {
 public:
  Listener() = default;
  void listenKeyboard();
  void stop() { isRunning_.store(false); }
  bool isRunning() const { return isRunning_.load(); }
  char getKeyboardInput() const { return key_input_.load(); }
  void clearKeyboardInput() { key_input_.store('\0'); }
  
 private:
  std::atomic<bool> isRunning_{true};
  std::atomic<char> key_input_ = {'\0'};
};

