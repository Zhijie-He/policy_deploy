#include <utility>
#include <iostream>
#include <thread>
#include <cstring>

#include "hardware/conio.h"
#include "hardware/listener.h"
#include "utility/logger.h"

void Listener::startKeyboardListener() {
  if (isRunning_.load()) return;

  isRunning_.store(true);
  keyboardThread_ = std::thread([this]() {
    FRC_INFO("[Listener.kbd] Press 'z' to exit keyboard listener.");
    while (isRunning()) {
      if (kbhit()) {
        auto c = getchar();
        if (c < 'Z' && c > 'A') c = c - 'A' + 'a';
        if (c != 10) {
          key_input_.store(c);
          FRC_INFO("[Listener.kbd] Pressed Button: " << c << ", " << key_input_.load());
        }
        if (c == 'z') {
          stop();
        }
      } else {
        usleep(10000);
      }
    }
    FRC_INFO("[Listener.kbd] Keyboard listener thread terminated.");
  });
}

void Listener::startJoystickListener(int id) {
  gamepad_.start(id);
}

void Listener::stop(){
  if (!isRunning_.load()) return;
  isRunning_.store(false);
  gamepad_.stop();
}

void Listener::join(){
  if (keyboardThread_.joinable()) {
    keyboardThread_.join();
  }
}

Listener::~Listener(){
  stop();
  join();
}

