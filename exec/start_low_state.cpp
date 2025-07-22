#include <GLFW/glfw3.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <csignal>
#include <atomic>
#include <wlrobot/robot/channel/channel_subscriber.hpp>
#include <wlrobot/robot/low_level/low_state_aggregator.hpp>
#include <wlrobot/idl/hd/IMUStatePubSubTypes.hpp>
#include <wlrobot/idl/hd/MotorStatesPubSubTypes.hpp>
#include <wlrobot/idl/hd/LowStatePubSubTypes.hpp>
using namespace wlrobot::robot;
using namespace wlrobot::msg;

#include "utility/tools.h"
#include "utility/timer.h"
#include "utility/cxxopts.hpp"

std::shared_ptr<BaseRobotConfig> cfg = nullptr;
std::atomic<bool> running(true);

void signal_handler(int) {
    running = false;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);  // 注册 Ctrl+C 处理
    cxxopts::Options options("robot sim sync", "Sync with robot and visualize using Mujoco");
    options.add_options()
        ("c,config", "Config name (e.g., mdl)", cxxopts::value<std::string>())
        ("h,help", "Print usage");

    auto result = options.parse(argc, argv);
    if (result.count("help") || !result.count("config")) {
        FRC_INFO(options.help());
        return 0;
    }
    std::string config_name = result["config"].as<std::string>();
    cfg = tools::loadConfig(config_name);

    //  start lowStateAggregator (combine imu and motor)
    auto lowstate_aggregator = std::make_unique<LowStateAggregator>(
        cfg->imu_topic,
        cfg->motor_topic,
        cfg->lowstate_topic,             // define by yourself
        cfg->domain_id,                // domain id
        800
    );

    lowstate_aggregator->Start();
     while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    lowstate_aggregator->Stop();  // 如果你实现了 Stop 方法
    return 0;
}
