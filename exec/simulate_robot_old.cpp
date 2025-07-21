
#include <thread>
#include <chrono>
#include <array>
#include <random>

#include <wlrobot/robot/channel/channel_publisher.hpp>
#include <wlrobot/idl/hd/IMUStatePubSubTypes.hpp>
#include <wlrobot/idl/hd/MotorStatesPubSubTypes.hpp>
using namespace wlrobot::robot;
using namespace wlrobot::msg;

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"
#include "utility/timer.h"

int main() {
    // 初始化 DDS 通道
    ChannelFactory::Instance()->Init(1);

    auto imu_pub = std::make_unique<ChannelPublisher<IMUState>>("/low_level/imu/state");
    imu_pub->InitChannel();

    auto motor_pub = std::make_unique<ChannelPublisher<MotorStates>>("/low_level/motor/state");
    motor_pub->InitChannel();

    FRC_INFO("Start publishing fake IMU and MotorStates data...");

    int64_t timestamp = 0;
    Timer simulateRobotTimer(0.01); // 100Hz

    // 随机数生成器
    std::default_random_engine generator(std::random_device{}());
    std::normal_distribution<float> noise_dist(0.0f, 0.02f);
    std::uniform_real_distribution<float> uniform_dist(-0.1f, 0.1f);

    while (true) {
        // === 构造 IMU 数据 ===
        IMUState imu;
        imu.timestamp(timestamp++);
        imu.temperature(36.5f + noise_dist(generator));
        imu.accelerometer({0.0f + noise_dist(generator),
                           0.0f + noise_dist(generator),
                           9.8f + noise_dist(generator)});
        imu.gyroscope({0.01f + noise_dist(generator),
                       0.02f + noise_dist(generator),
                       0.03f + noise_dist(generator)});
        imu.quaternion({1.0f, 
                        0.0f + noise_dist(generator), 
                        0.0f + noise_dist(generator), 
                        0.0f + noise_dist(generator)});
        imu.rpy({0.1f + noise_dist(generator),
                 0.2f + noise_dist(generator),
                 0.3f + noise_dist(generator)});

        // === 构造 MotorStates 数据 ===
        MotorStates motors;

        std::array<float, 12> pos{}, vel{}, tau{};
        std::array<int, 12> temp{};
        std::array<uint32_t, 12> mode{};
        for (int i = 0; i < 12; ++i) {
            pos[i] = 0.1f * i + uniform_dist(generator);
            vel[i] = 0.01f * i + noise_dist(generator);
            tau[i] = 0.001f * i + noise_dist(generator);
            temp[i] = 36.0f + noise_dist(generator);
            mode[i] = i % 5;
        }

        motors.pos(pos);
        motors.w(vel);
        motors.t(tau);
        motors.temperature(temp);
        motors.mode(mode);

        // 发布
        imu_pub->Write(imu);
        motor_pub->Write(motors);

        FRC_INFO("[MockPub] Published timestamp = " << timestamp);
        simulateRobotTimer.wait();
    }

    return 0;
}
