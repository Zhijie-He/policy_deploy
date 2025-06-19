#pragma once

#include <cassert>
#include <cstdint>
#include <ctime>
#include <cmath>
#include <unistd.h>
#include <chrono>
#include <string>
#include <thread>
#include <iostream>
#include "utility/logger.h"

using namespace std::chrono;

class Timer {
public:
  Timer() = default;
  explicit Timer(const double dt): algo_period(dt) { start(); }
  ~Timer() = default;

  void stop() const {
    // macOS 没有 timerFd，可以忽略
  }

  void start() {
    clock_gettime(CLOCK_MONOTONIC, &_startTime);
    // macOS 不支持 timerfd_create，所以不设置周期性定时器
    tick_time = getNs();
  }

  void wait() {
    // macOS 不支持 timerfd -> 忽略
    usleep(static_cast<useconds_t>(algo_period * 1e6));  // 简单替代
  }

  double getElapsedTime() {
    auto currentTime = getNs();
    auto delta = (currentTime - tick_time) * 1e-9;
    tick_time = currentTime;
    return delta;
  }

  double getElapsedTime_us() {
    auto currentTime = getNs();
    auto delta = (currentTime - tick_time) * 1e-3;
    tick_time = currentTime;
    return delta;
  }

  int64_t getNs() const {
    struct timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)(now.tv_nsec - _startTime.tv_nsec) +
           1000000000 * (now.tv_sec - _startTime.tv_sec);
  }

  double getMs() const { return static_cast<double>(getNs()) / 1.e6; }
  double getUs() const { return static_cast<double>(getNs()) / 1.e3; }
  double getSeconds() const { return static_cast<double>(getNs()) / 1.e9; }

  int64_t tick_time{0};
  struct timespec _startTime{};
  double algo_period{0};
};

inline uint64_t timeSinceEpochUs() {
  return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

class RateLimiter {
public:
    using clock = std::chrono::steady_clock;
    using nanoseconds = std::chrono::nanoseconds;

    RateLimiter(double frequency_hz = 100.0, std::string name = "loop", bool warn = true)
        : period_ns_(static_cast<int64_t>(1e9 / frequency_hz)),
          name_(std::move(name)), warn_(warn)
    {
        reset();
    }

    void reset() {
        start_time_ = clock::now();
        next_time_ = start_time_ + period_ns_;
    }

    void wait() {
        auto now = clock::now();
        if (now < next_time_) {
            std::this_thread::sleep_until(next_time_);
        } else if (warn_) {
            auto late_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - next_time_).count();
            FRC_WARN("[RateLimiter] [" << name_ << "] is late by " << late_ms << " [ms]");
        }
        next_time_ += period_ns_;
    }

    double timeSinceStartMs() const {
        return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(clock::now() - start_time_).count();
    }

    double timeSinceStartS() const {
        return std::chrono::duration_cast<std::chrono::duration<double>>(clock::now() - start_time_).count();
    }

private:
    nanoseconds period_ns_;
    std::chrono::time_point<clock> start_time_;
    std::chrono::time_point<clock> next_time_;
    std::string name_;
    bool warn_;
};

