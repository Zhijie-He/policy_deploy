#include <wlrobot/robot/low_level/low_state_aggregator.hpp>

using namespace wlrobot::robot;

void LowStateAggregator::imu_callback(const wlrobot::msg::IMUState& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_imu_ = msg;
    has_imu_ = true;
}

void LowStateAggregator::motor_callback(const wlrobot::msg::MotorStates& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_motor_ = msg;
    has_motor_ = true;
}

void LowStateAggregator::Start() {
    ChannelFactory::Instance()->Init(domain_id_);

    imu_sub_ = std::make_unique<ChannelSubscriber<wlrobot::msg::IMUState>>(imu_topic_);
    motor_sub_ = std::make_unique<ChannelSubscriber<wlrobot::msg::MotorStates>>(motor_topic_);
    lowstate_pub_ = std::make_unique<ChannelPublisher<wlrobot::msg::LowState>>(lowstate_topic_);

    imu_sub_->InitChannel([this](const wlrobot::msg::IMUState& msg) { imu_callback(msg); });
    motor_sub_->InitChannel([this](const wlrobot::msg::MotorStates& msg) { motor_callback(msg); });
    lowstate_pub_->InitChannel();

    running_ = true;
    publish_thread_ = std::thread(&LowStateAggregator::publish_loop, this);
}

void LowStateAggregator::publish_loop() {
    int64_t tick = 0;
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(publish_interval_ms_));

        if (has_imu_ && has_motor_) {
            std::lock_guard<std::mutex> lock(mutex_);
            wlrobot::msg::LowState state;
            state.tick() = tick++;
            state.imu_state() = latest_imu_;
            state.motor_state() = latest_motor_;
            lowstate_pub_->Write(state);
        }
    }
}

