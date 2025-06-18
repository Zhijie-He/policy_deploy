/*! @file Timer.h
 *  @brief Timer for measuring how long things take
 */

#pragma once

#include <cassert>
#include <cstdint>
#include <ctime>
#include <cmath>
#include <unistd.h>
#include "sys/timerfd.h"

/*!
 * Timer for measuring time elapsed with clock_monotonic
 */
class Timer {
public:

  /*!
   * Construct and start timer
   * Initialize with no argument means you only use this class for tick timing.
   * Initialize with period argument gives you access to set a timed loop.
   */

  Timer() = default;

  explicit Timer(const double dt): algo_period(dt) { start(); }

  ~Timer() { stop(); }

  void stop() const {close(timerFd);}

  void start() {
    clock_gettime(CLOCK_MONOTONIC, &_startTime);
    if (algo_period > 0) {
      timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
      int seconds = (int) algo_period;
      int nanoseconds = (int) (1e9 * fmod(algo_period, 1.f));
      missed = 0;
      timerSpec.it_interval.tv_sec = seconds;
      timerSpec.it_interval.tv_nsec = nanoseconds;
      timerSpec.it_value.tv_sec = seconds;
      timerSpec.it_value.tv_nsec = nanoseconds;
      ret = timerfd_settime(timerFd, 0, &timerSpec, nullptr);
    }
  }

  void wait() {
    if (algo_period > 0) {
      m = read(timerFd, &missed, sizeof(missed));
      (void) m;
    }
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

  /*!
   * Get nanoseconds elapsed
   */
  int64_t getNs() const {
    struct timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t) (now.tv_nsec - _startTime.tv_nsec) +
           1000000000 * (now.tv_sec - _startTime.tv_sec);
  }

  /*!
 * Get milliseconds elapsed
 */
  double getMs() const { return static_cast<double>(getNs()) / 1.e6; }

  /*!
 * Get microsecond elapsed
 */
  double getUs() const { return static_cast<double>(getNs()) / 1.e3; }

  /*!
   * Get seconds elapsed
   */
  double getSeconds() const { return static_cast<double>(getNs()) / 1.e9; }

  int64_t tick_time{0};
  struct timespec _startTime{};
  double algo_period{0};
  itimerspec timerSpec{};
  unsigned long long missed{};
  int timerFd{};
  int ret{};
  ssize_t m{};
};

