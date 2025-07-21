// sim2/real/Sim2WlRobotEnv.cpp
#include <thread>
#include "sim2/real/sim2wlrobot_env.h"

Sim2WlRobotEnv::Sim2WlRobotEnv(const std::string& net_interface,
                               std::shared_ptr<const BaseRobotConfig> cfg,
                               std::shared_ptr<StateMachine> state_machine)
    : BaseEnv(cfg, state_machine),
      net_interface_(net_interface)
{   
  FRC_INFO("[Sim2WlRobotEnv.Const] net_interface: " << net_interface);
  ChannelFactory::Instance()->Init(cfg->domain_id);

  // create publisher
  lowcmd_publisher_ = std::make_unique<ChannelPublisher<MotorCmds>>(cfg_->lowcmd_topic);
  lowcmd_publisher_->InitChannel();
  
  // create subscriber
  lowstate_subscriber_ = std::make_unique<ChannelSubscriber<LowState>>(cfg_->lowstate_topic);
  lowstate_subscriber_->InitChannel(
    [this](const LowState& msg) {
      this->LowStateHandler(msg);
    }, 10
  );
  
  initState();
  waitForLowState();  // wait for the subscriber to receive data
}

void Sim2WlRobotEnv::waitForLowState() {
  FRC_INFO("[Sim2WlRobotEnv.waitForLowState] Waiting to connect to the robot.");
  while (low_state_buffer_.GetCopy().tick() == 0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(control_dt_));
  }
  FRC_INFO("[Sim2WlRobotEnv.waitForLowState] Successfully connected to the robot.");
}

void Sim2WlRobotEnv::LowStateHandler(const LowState& message) {
  low_state_buffer_.SetData(message);
  updateRobotState();
}

void Sim2WlRobotEnv::updateRobotState() {
  LowState state = low_state_buffer_.GetCopy();

  // === 获取广义坐标 (generalized coordinates) ===
  Eigen::Vector3f rootPos = Eigen::Vector3f::Zero();
  Eigen::Vector4f rootQuat;  
  rootQuat << state.imu_state().quaternion()[0],
              state.imu_state().quaternion()[1],
              state.imu_state().quaternion()[2],
              state.imu_state().quaternion()[3];
  Eigen::VectorXf jointPos(jointDim_);
  for (int i = 0; i < jointDim_; ++i) {
    jointPos[i] = state.motor_state().pos()[i];
  }

  // === 获取速度项 ===
  Eigen::Vector3f rootVel = Eigen::Vector3f::Zero(); 
  Eigen::Vector3f rootAngVel;
  rootAngVel << state.imu_state().gyroscope()[0],
                state.imu_state().gyroscope()[1],
                state.imu_state().gyroscope()[2];
  Eigen::VectorXf jointVel(jointDim_);
  for (int i = 0; i < jointDim_; ++i) {
    jointVel[i] =  state.motor_state().w()[i];
  }

  gc_ << rootPos, rootQuat, jointPos;
  gv_ << rootVel, rootAngVel, jointVel;
  
  // === 更新 robotStatus 共享结构 ===
  if (robotStatusBufferPtr_) {
    robotStatus status;
    memcpy(status.data.position, gc_.data(), gcDim_ * sizeof(float));
    memcpy(status.data.velocity, gv_.data(), gvDim_ * sizeof(float));
    status.data.timestamp = state.tick();  
    robotStatusBufferPtr_->SetData(status);
  }
}

void Sim2WlRobotEnv::sendCmd(MotorCmds& cmd) {
  lowcmd_publisher_->Write(cmd);        
}

void Sim2WlRobotEnv::zeroTorqueState() {
  FRC_INFO("[Sim2WlRobotEnv.zeroTorqueState] Enter zero torque state.");
  FRC_INFO("[Sim2WlRobotEnv.zeroTorqueState] Waiting for the start signal...");
  Timer zeroTorqueStateTimer(control_dt_);

  while (listenerPtr_ && listenerPtr_->gamepad_.start.pressed != 1) {
    sendCmd(low_cmd_);        
    zeroTorqueStateTimer.wait();
  }
}

void Sim2WlRobotEnv::moveToDefaultPos() {
  FRC_INFO("[Sim2WlRobotEnv.moveToDefaultPos] Moving to default position...");
  Timer moveToDefaultPosTimer(control_dt_);

  const float total_time = 2.0f;
  int num_step = static_cast<int>(total_time / control_dt_);

  // config 参数
  const auto& kps = cfg_->kP;
  const auto& kds = cfg_->kD;
  const auto& default_joint_pos = cfg_->default_angles;
  int dof_size = jointDim_;

  // 初始位置
  std::vector<float> init_dof_pos(dof_size, 0.0f);
  LowState state = low_state_buffer_.GetCopy();
  for (int i = 0; i < dof_size; ++i) {
      init_dof_pos[i] = state.motor_state().pos()[i];
  }

  // move to default pos
  for (int i = 0; i < num_step; ++i) {
      float alpha = static_cast<float>(i) / num_step;
      for (int j = 0; j < dof_size; ++j) {
          int motor_idx = j;
          float target_q = init_dof_pos[motor_idx] * (1.0f - alpha) + default_joint_pos[motor_idx] * alpha;
          low_cmd_.pos()[motor_idx] = target_q;
          low_cmd_.w()[motor_idx] = 0;
          low_cmd_.kp()[motor_idx] = kps[motor_idx];
          low_cmd_.kd()[motor_idx] = kds[motor_idx];
          low_cmd_.t()[motor_idx]= 0;
      }
      sendCmd(low_cmd_);
      moveToDefaultPosTimer.wait();
  }

  FRC_INFO("[Sim2WlRobotEnv.moveToDefaultPos] Reached default position.");
}

void Sim2WlRobotEnv::defaultPosState() {
  FRC_INFO("[Sim2WlRobotEnv.defaultPosState] Enter default pos state.");
  FRC_INFO("[Sim2WlRobotEnv.defaultPosState] Waiting for the Button A signal...");

  Timer defaultPosStateTimer(control_dt_);

  // config 参数
  const auto& kps = cfg_->kP;
  const auto& kds = cfg_->kD;
  const auto& default_joint_pos = cfg_->default_angles;
  int dof_size = jointDim_;

  while (listenerPtr_ && listenerPtr_->gamepad_.A.pressed != 1) {
    for (int i = 0; i < dof_size; ++i) {
      int motor_idx = i;
      low_cmd_.pos()[motor_idx] = default_joint_pos[motor_idx];
      low_cmd_.w()[motor_idx] = 0;
      low_cmd_.kp()[motor_idx] = kps[motor_idx];
      low_cmd_.kd()[motor_idx] = kds[motor_idx];
      low_cmd_.t()[motor_idx]= 0;
    }
    sendCmd(low_cmd_);
    defaultPosStateTimer.wait();
  }
}

void Sim2WlRobotEnv::run() {
  RateLimiter controlTimer(1.0 / control_dt_, "real main loop");
  
  while (running_) {
    counter_++;

    // emergency stop button
    if (listenerPtr_ && listenerPtr_->gamepad_.select.pressed == 1) {
      FRC_INFO("[Sim2WlRobotEnv.run] Emergency Stop! at " << counter_ << "count");
      running_ = false;
      break;
    }
    
    if(state_machine_) {
      auto t_start = std::chrono::high_resolution_clock::now();
      state_machine_->step();
      auto t_end = std::chrono::high_resolution_clock::now();
      
      double run_time_us = std::chrono::duration<double, std::milli>(t_end - t_start).count();
      run_sum_us += run_time_us;
      run_sum_sq_us += run_time_us * run_time_us;
      ++run_count;

      if (run_count % 100 == 0) {
          double avg = run_sum_us / run_count;
          double stddev = std::sqrt(run_sum_sq_us / run_count - avg * avg);
          FRC_INFO("[StateMachine.step] Step 100 runs AVG: " << avg << " ms | STDDEV: " << stddev << " ms");
          
          // 重置
          run_sum_us = 0;
          run_sum_sq_us = 0;
          run_count = 0;
      }
    }
    
    auto actionPtr = jointCMDBufferPtr_->GetData();  
    while (!actionPtr) { 
      FRC_INFO("[Sim2WlRobotEnv.run] Waiting For actions!");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      actionPtr = jointCMDBufferPtr_->GetData(); 
    }

    {
      std::lock_guard<std::mutex> actionLock(action_lock_);
      const auto& cmd = *actionPtr;
      memcpy(pTarget.data(), cmd.data.position, sizeof(float) * jointDim_);
      memcpy(vTarget.data(), cmd.data.velocity, sizeof(float) * jointDim_);
      memcpy(jointPGain.data(), cmd.data.kp, sizeof(float) * jointDim_);
      memcpy(jointDGain.data(), cmd.data.kd, sizeof(float) * jointDim_);

      for (int i = 0; i < jointDim_; ++i) {
        low_cmd_.pos()[i]  = pTarget[i];
        low_cmd_.w()[i]    = vTarget[i];
        low_cmd_.kp()[i]   = jointPGain[i];
        low_cmd_.kd()[i]   = jointDGain[i];
        low_cmd_.t()[i]    = 0.0f;
      }
    }

    sendCmd(low_cmd_);
    controlTimer.wait();
  }
}



