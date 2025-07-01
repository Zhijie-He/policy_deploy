#include "hardware/listener.h"
#include "hardware/conio.h"
#include "utility/logger.h"
#include <iostream>
#include <thread>
#include <cctype>  // for std::tolower
#include <stdexcept>

Listener::~Listener() {
  stop();
  join();
}

void Listener::start() {
  if (isRunning_) return;  // 防止重复启动
  isRunning_ = true;
  keyboard_thread_ = std::thread(&Listener::listenKeyboard, this);
}

void Listener::stop() {
  isRunning_ = false;
}

void Listener::join() {
  if (keyboard_thread_.joinable())
    keyboard_thread_.join();
}

void Listener::listenKeyboard() {
  FRC_INFO("[Listener.kbd] Press 'z' to exit keyboard listener.");
  while (isRunning()) {
    if (kbhit()) {
      char c = getchar();

      // 转小写（安全）
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

      if (c != '\n') {
        key_input_ = c;
        FRC_INFO("[Listener.kbd] Pressed Button: " << c);

        // 如果是 'z'，先退出
        if (c == 'z') {
          FRC_INFO("[Listener.kbd] Received 'z', stopping listener.");
          stop();
          break;
        }

        // 安全触发回调
        try {
          if (key_callback_) {
            key_callback_(c);
          }
        } catch (const std::exception& e) {
          FRC_ERROR("[Listener.kbd] Exception in key callback: " << e.what());
        }
      }
    } else {
      usleep(10000);  // 减少 CPU 占用
    }
  }
  FRC_INFO("[Listener.kbd] Keyboard listening thread terminated.");
}

