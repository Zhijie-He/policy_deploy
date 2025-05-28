#pragma once

#include <iostream>
#include <sstream>
#include <ctime>
#include <iomanip>

// 🧠 是否启用彩色输出
#ifndef LOG_USE_COLOR
#define LOG_USE_COLOR 1
#endif

// 🧠 是否启用前缀 [INFO time file:line]
#ifndef LOG_USE_PREFIX
#define LOG_USE_PREFIX 1
#endif

// 🧠 是否启用 DEBUG 输出（不定义则不会打印 DBG_INFO）
#ifdef PRINT_INFO
#define LOG_ENABLE_DEBUG 1
#else
#define LOG_ENABLE_DEBUG 0
#endif

// 🎨 彩色控制字符（根据开关选择）
#if LOG_USE_COLOR
  #define COLOR_RESET   "\033[0m"
  #define COLOR_INFO    "\033[1;32m"
  #define COLOR_WARN    "\033[1;33m"
  #define COLOR_ERROR   "\033[1;31m"
  #define COLOR_DEBUG   "\033[1;34m"
#else
  #define COLOR_RESET   ""
  #define COLOR_INFO    ""
  #define COLOR_WARN    ""
  #define COLOR_ERROR   ""
  #define COLOR_DEBUG   ""
#endif

// 🕒 当前时间字符串
inline std::string current_time_str() {
    std::time_t now = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%H:%M:%S");
    return ss.str();
}

// 📦 日志前缀
#if LOG_USE_PREFIX
  #define FRC_PREFIX(level, color) color "[" level " " << current_time_str() << " " << __FILE__ << ":" << __LINE__ << "] "
#else
  #define FRC_PREFIX(level, color) color
#endif

// ✅ 日志宏定义
#define FRC_INFO(x)  std::cout << FRC_PREFIX("INFO",  COLOR_INFO)  << x << COLOR_RESET << std::endl
#define FRC_WARN(x)  std::cout << FRC_PREFIX("WARN",  COLOR_WARN)  << x << COLOR_RESET << std::endl
#define FRC_ERROR(x) std::cerr << FRC_PREFIX("ERROR", COLOR_ERROR) << x << COLOR_RESET << std::endl

#if LOG_ENABLE_DEBUG
  #define DBG_INFO(x)  std::cout << FRC_PREFIX("DEBUG", COLOR_DEBUG) << x << COLOR_RESET << std::endl
#else
  #define DBG_INFO(x)
#endif
