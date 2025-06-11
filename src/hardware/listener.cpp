#include <utility>
#include <iostream>
#include <thread>
#include <cstring>
#include "hardware/listener.h"
#include "hardware/conio.h"
#include "utility/logger.h"

void Listener::listenKeyboard() {
  FRC_INFO("[Listener.kbd] Press 'z' to exit after saving run-time data.");
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

// 解析 unitree wireless_remote 数据（40字节）
void RemoteController::set(const uint8_t* data) {
  // button bits in byte[2:4]
  uint16_t keys = data[2] | (data[3] << 8);
  for (int i = 0; i < 16; ++i)
    button[i] = (keys >> i) & 1;

  std::memcpy(&lx, data + 4, 4);
  std::memcpy(&rx, data + 8, 4);
  std::memcpy(&ry, data + 12, 4);
  std::memcpy(&ly, data + 20, 4);
}

