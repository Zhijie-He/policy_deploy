#pragma once

#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>
#include <memory>

#include <wlrobot/robot/channel/channel_subscriber.hpp>
#include <wlrobot/robot/channel/channel_publisher.hpp>
#include <wlrobot/robot/channel/channel_subscriber.hpp>
#include <wlrobot/idl/hd/IMUStatePubSubTypes.hpp>
#include <wlrobot/idl/hd/MotorStatesPubSubTypes.hpp>
#include <wlrobot/idl/hd/LowStatePubSubTypes.hpp>

namespace wlrobot::robot {

class LowStateAggregator {
public:
    LowStateAggregator(const std::string& imu_topic,
                       const std::string& motor_topic,
                       const std::string& lowstate_topic,
                       int domain_id,
                       int pub_hz = 500)
        : imu_topic_(imu_topic),
          motor_topic_(motor_topic),
          lowstate_topic_(lowstate_topic),
          publish_interval_ms_(1000 / pub_hz),
          domain_id_(domain_id)
    {}

    void Start();
    void Stop();
   

private:
    int domain_id_;
    void imu_callback(const wlrobot::msg::IMUState& msg);
    void motor_callback(const wlrobot::msg::MotorStates& msg);
    void process_motor_data(const wlrobot::msg::MotorStates& msg);
    void publish_loop();

    std::string imu_topic_, motor_topic_, lowstate_topic_;
    int publish_interval_ms_;

    std::mutex mutex_;
    wlrobot::msg::IMUState latest_imu_;
    wlrobot::msg::MotorStates latest_motor_;
    std::atomic<bool> has_imu_{false}, has_motor_{false};

    std::unique_ptr<ChannelSubscriber<wlrobot::msg::IMUState>> imu_sub_;
    std::unique_ptr<ChannelSubscriber<wlrobot::msg::MotorStates>> motor_sub_;
    std::unique_ptr<ChannelPublisher<wlrobot::msg::LowState>> lowstate_pub_;

    std::thread publish_thread_;
    std::atomic<bool> running_{false};
    
    //related to calibartion
    std::array<float, 4> abad_sign_  = { 1.0f,  1.0f, -1.0f, -1.0f };
    std::array<float, 4> hip_sign_   = {-1.0f,  1.0f, -1.0f,  1.0f };
    std::array<float, 4> knee_sign_  = {-1.0f,  1.0f, -1.0f,  1.0f };
    std::array<std::array<float, 3>, 4> zero_offset_ = {{
        { 0.882785f,  1.56933f,  -3.15066f },
        {-0.245677f, -0.873517f,  2.25232f },
        { 0.169374f,  1.31086f,  -2.8007f  },
        { 0.422843f, -0.535587f,  2.45248f }
    }};
    std::array<int, 12> real2sim_dof_map_ = { 3,4,5, 0,1,2, 9,10,11, 6,7,8 };

};

}  // namespace wlrobot::robot

