#pragma once

#include <atomic>
#include <cstdint>
#include "gamepad.hpp"

// KeyMap 按键编号
struct KeyMap {
  static constexpr int R1 = 0, L1 = 1, start = 2, select = 3;
  static constexpr int R2 = 4, L2 = 5, F1 = 6, F2 = 7;
  static constexpr int A = 8, B = 9, X = 10, Y = 11;
  static constexpr int up = 12, right = 13, down = 14, left = 15;
};

// 遥控器结构
struct RemoteController {
  float lx = 0.f, ly = 0.f, rx = 0.f, ry = 0.f;
  uint8_t button[16] = {0};

  void set(const uint8_t* data);  // 解析 wireless_remote[40] 数据
};

// 键盘与遥控器监听器
class Listener {
 public:
  Listener() = default;
  void listenKeyboard();
  void stop() { isRunning_.store(false); }
  bool isRunning() const { return isRunning_.load(); }
  char* getKeyInputPtr() { return &key_input_; }
  
  RemoteController remote;
  
  // unitree official
  unitree::common::Gamepad gamepad_;
  unitree::common::REMOTE_DATA_RX rx_;

 private:
  std::atomic<bool> isRunning_{true};
  char key_input_ = '\0';
};

