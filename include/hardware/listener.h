#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <thread>
#include "gamepad.hpp"

// 键盘与遥控器监听器
class Listener {
 public:
  Listener() = default;
  ~Listener();

  void start();                // 启动监听线程
  void stop();                 // 请求退出
  void join();                 // 等待线程退出
  bool isRunning() const { return isRunning_.load(); }

  char* getKeyInputPtr() { return &key_input_; }

  // 注册键盘回调
  void setKeyboardCallback(std::function<void(char)> cb) {
    key_callback_ = std::move(cb);
  }

  // unitree 官方遥控器结构
  unitree::common::Gamepad gamepad_;
  unitree::common::REMOTE_DATA_RX rx_;

 private:
  void listenKeyboard();  // 监听线程主函数

  std::atomic<bool> isRunning_{false};
  std::thread keyboard_thread_;
  char key_input_ = '\0';

  std::function<void(char)> key_callback_;
};

