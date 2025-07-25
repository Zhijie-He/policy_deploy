#pragma once

#include <SDL2/SDL.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <unordered_map>
#include <string>
#include <cmath>
#include <chrono>
#include <algorithm>

// bytecode mapping for raw joystick data
// 16b
typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t back : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t L3 : 1;
        uint8_t R3 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } bits;
    uint16_t value = 0;
} JoystickKeyUnion;


// 40 Byte (now used 24B)
typedef struct
{
  JoystickKeyUnion buttons;
  float lx = 0.0f, ly = 0.0f;
  float rx = 0.0f, ry = 0.0f;
  float lt = 0.0f, rt = 0.0f;
} JoystickRawData;

class Button
{
public:
    void update(bool current)
    {
        on_press = current && !pressed;
        on_release = !current && pressed;
        pressed = current;
    }

    bool pressed = false;
    bool on_press = false;
    bool on_release = false;
};

class Joystick {
public:
    void update(const JoystickRawData& raw) {
        lx = filter(lx, raw.lx);
        ly = filter(ly, raw.ly);
        rx = filter(rx, raw.rx);
        ry = filter(ry, raw.ry);
        lt = filter(lt, raw.lt);
        rt = filter(rt, raw.rt);

        A.update(raw.buttons.bits.A);
        B.update(raw.buttons.bits.B);
        X.update(raw.buttons.bits.X);
        Y.update(raw.buttons.bits.Y);
        L1.update(raw.buttons.bits.L1);
        R1.update(raw.buttons.bits.R1);
        L3.update(raw.buttons.bits.L3);
        R3.update(raw.buttons.bits.R3);
        start.update(raw.buttons.bits.start);
        back.update(raw.buttons.bits.back);
        up.update(raw.buttons.bits.up);
        down.update(raw.buttons.bits.down);
        left.update(raw.buttons.bits.left);
        right.update(raw.buttons.bits.right);
    }

    float lx = 0.0f, ly = 0.0f;
    float rx = 0.0f, ry = 0.0f;
    float lt = 0.0f, rt = 0.0f;

    float dead_zone = 0.01f;
    float smooth = 0.03f;

    Button A, B, X, Y;
    Button L1, R1, L3, R3;
    Button start, back;
    Button up, down, left, right;

private:
    float filter(float prev, float curr) {
        return (std::fabs(curr) < dead_zone) ? prev * (1 - smooth) : prev * (1 - smooth) + curr * smooth;
    }
};

class JoystickService {
 public:
  void start(int controller_id = 0) {
    if (running_) return;

    if (SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER) != 0) {
      std::cerr << "[Gamepad] SDL_Init failed: " << SDL_GetError() << "\n";
      return;
    }
    std::string project_source_dir = PROJECT_SOURCE_DIR;
    std::string gamecontrollerdb_path = project_source_dir + "/include/hardware/gamecontrollerdb.txt";
    SDL_GameControllerAddMappingsFromFile(gamecontrollerdb_path.c_str());

    running_ = true;
    std::thread([this, controller_id] { loop(controller_id); }).detach();
  }

  void stop() { running_ = false; }
  
  bool isConnected() const { return connected_.load(); }

  Joystick getJoystick() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return joy_;
  }
  
 private:
  void loop(int controller_id) {
    SDL_GameController* controller = SDL_GameControllerOpen(controller_id);
    if (!controller) {
      std::cerr << "[Gamepad] Failed to open controller: " << SDL_GetError() << "\n";
      running_ = false;
      connected_ = false;
      return;
    }
    connected_ = true;

    SDL_Event e;
    while (running_) {
      while (SDL_PollEvent(&e)) {
        processEvent(e);  // 处理按钮事件
      }

      // 每帧主动刷新轴状态，解决回中无响应的问题
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto norm = [](int16_t v) {
          constexpr float MAX = 32767.0f;
          return std::clamp(static_cast<float>(v) / MAX, -1.0f, 1.0f);
        };
        raw_data_.lx = norm(SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX));
        raw_data_.ly = norm(SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY));
        raw_data_.rx = norm(SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTX));
        raw_data_.ry = norm(SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTY));
        raw_data_.lt = norm(SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT));
        raw_data_.rt = norm(SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT));
      }

      joy_.update(raw_data_);
      SDL_Delay(10);
    }
    
    SDL_GameControllerClose(controller);
  }

  void processEvent(const SDL_Event& e) {
    if (e.type != SDL_CONTROLLERBUTTONDOWN && e.type != SDL_CONTROLLERBUTTONUP) return;

    bool pressed = (e.type == SDL_CONTROLLERBUTTONDOWN);

    std::lock_guard<std::mutex> lock(mutex_);
    auto& b = raw_data_.buttons.bits;

    switch (e.cbutton.button) {
      case SDL_CONTROLLER_BUTTON_A:            b.A = pressed;      break;
      case SDL_CONTROLLER_BUTTON_B:            b.B = pressed;      break;
      case SDL_CONTROLLER_BUTTON_X:            b.X = pressed;      break;
      case SDL_CONTROLLER_BUTTON_Y:            b.Y = pressed;      break;
      case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: b.L1 = pressed;      break;
      case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:b.R1 = pressed;      break;
      case SDL_CONTROLLER_BUTTON_LEFTSTICK:    b.L3 = pressed;      break;
      case SDL_CONTROLLER_BUTTON_RIGHTSTICK:   b.R3 = pressed;      break;
      case SDL_CONTROLLER_BUTTON_START:        b.start = pressed;   break;
      case SDL_CONTROLLER_BUTTON_BACK:         b.back = pressed;  break;
      case SDL_CONTROLLER_BUTTON_DPAD_UP:      b.up = pressed;      break;
      case SDL_CONTROLLER_BUTTON_DPAD_DOWN:    b.down = pressed;    break;
      case SDL_CONTROLLER_BUTTON_DPAD_LEFT:    b.left = pressed;    break;
      case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:   b.right = pressed;   break;
      default:
        break; // 忽略不处理的按键
    }
  }


  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  mutable std::mutex mutex_;
  JoystickRawData raw_data_;
  Joystick joy_;
};
