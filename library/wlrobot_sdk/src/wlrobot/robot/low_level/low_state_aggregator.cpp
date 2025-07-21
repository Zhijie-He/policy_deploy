#include <wlrobot/robot/low_level/low_state_aggregator.hpp>
using namespace wlrobot::robot;

void LowStateAggregator::imu_callback(const wlrobot::msg::IMUState& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_imu_ = msg;
    has_imu_ = true;
}

void LowStateAggregator::motor_callback(const wlrobot::msg::MotorStates& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    // process_motor_data(msg);
    latest_motor_ = msg;
    has_motor_ = true;
}

void LowStateAggregator::process_motor_data(const wlrobot::msg::MotorStates& msg) {
    const auto& pos_in  = msg.pos();
    const auto& vel_in  = msg.w();
    const auto& tau_in  = msg.t();
    const auto& temp_in = msg.temperature();
    const auto& mode_in = msg.mode();

    float real_pos[12], real_vel[12], real_tau[12];
    int real_temp[12], real_mode[12];

    for (int i = 0; i < 12; ++i) {
        int leg = i / 3;
        int joint = i % 3;

        float sign = (joint == 0) ? abad_sign_[leg] :
                        (joint == 1) ? hip_sign_[leg] :
                                       knee_sign_[leg];
        float offset = zero_offset_[leg][joint];

        real_pos[i]  = (pos_in[i] - offset) * sign;
        real_vel[i]  = vel_in[i] * sign;
        real_tau[i]  = tau_in[i] * sign;
        real_temp[i] = temp_in[i];
        real_mode[i] = mode_in[i];
    }

    auto& pos_out  = latest_motor_.pos();
    auto& vel_out  = latest_motor_.w();
    auto& tau_out  = latest_motor_.t();
    auto& temp_out = latest_motor_.temperature();
    auto& mode_out = latest_motor_.mode();

    for (int sim_idx = 0; sim_idx < 12; ++sim_idx) {
        int real_idx = real2sim_dof_map_[sim_idx];
        pos_out[sim_idx]  = real_pos[real_idx];
        vel_out[sim_idx]  = real_vel[real_idx];
        tau_out[sim_idx]  = real_tau[real_idx];
        temp_out[sim_idx] = real_temp[real_idx];
        mode_out[sim_idx] = real_mode[real_idx];
    }
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

void LowStateAggregator::Stop() {
    running_ = false;
    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }
}

void LowStateAggregator::publish_loop() {
    int64_t tick = 0;
    wlrobot::msg::LowState state;

    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(publish_interval_ms_));

        if (has_imu_ && has_motor_) {
            std::lock_guard<std::mutex> lock(mutex_);
            state.tick() = tick++;
            state.imu_state() = latest_imu_;
            state.motor_state() = latest_motor_;
            lowstate_pub_->Write(state);
        }
    }
}
