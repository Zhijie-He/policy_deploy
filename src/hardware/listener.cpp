#include <utility>
#include <iostream>
#include <thread>
#include <cstring>
#include "hardware/listener.h"
#include "hardware/conio.h"
#include "utility/logger.h"

void Listener::listenKeyboard() {
  FRC_INFO("[Listener.kbd] Press 'z' to exit keyboard listener.");
  while (isRunning()) {
    if (kbhit()) {
      auto c = getchar();
      if (c < 'Z' && c > 'A') c = c - 'A' + 'a';
      if (c != 10) {
        key_input_ = c;
        FRC_INFO("[Listener.kbd] Pressed Button: " << c << ", " << key_input_);
      }
      switch (c) {
        case 'z':
          stop();  // 原子安全停止
          break;
        case '\n':
          break;
        default:
          break;
      }
    } else {
      usleep(10000);
    }
  }
  FRC_INFO("[Listener.kbd] Keyboard listening thread terminated.");
}

