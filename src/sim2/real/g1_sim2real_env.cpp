// G1Sim2RealEnv.cpp
#include "sim2/real/g1_sim2real_env.h"
#include "utility/logger.h"
#include <thread>
#include "utility/timer.h"

G1Sim2RealEnv::G1Sim2RealEnv(const std::string& net_interface,
                            std::shared_ptr<const BaseRobotConfig> cfg,
                            std::shared_ptr<StateMachine> state_machine)
    : BaseEnv(cfg, state_machine),
      net_interface_(net_interface),
      mode_pr_(Mode::PR),
      mode_machine_(0)
{   
    FRC_INFO("[G1Sim2RealEnv.Const] net_interface: " << net_interface);

    // initialize DDS communication
    // ChannelFactory::Instance()->Init(0, net_interface.c_str()); 
    ChannelFactory::Instance()->Init(0);

    if(cfg_->msg_type == "hg"){
      // create publisher
      lowcmd_publisher_ = std::make_unique<ChannelPublisher<LowCmd_>>(cfg_->lowcmd_topic);
      lowcmd_publisher_->InitChannel();
      
      // create subscriber
      lowstate_subscriber_ = std::make_unique<ChannelSubscriber<LowState_>>(cfg_->lowstate_topic);
      lowstate_subscriber_->InitChannel(
        [this](const void *message) {
          this->LowStateHandler(message);
        }, 10
      );
    } else {
      FRC_ERROR("[G1Sim2RealEnv.Const] Invalid msg_type: " << cfg_->msg_type);
      throw std::invalid_argument("Invalid msg_type: " + cfg_->msg_type);
    }
    
    initState();
    
    // wait for the subscriber to receive data
    waitForLowState();

    // Initialize the command msg
    unitree_tools::init_cmd_hg(low_cmd_, mode_machine_, mode_pr_);

    unitree_tools::print_lowcmd(low_cmd_);
}

void G1Sim2RealEnv::waitForLowState() {
    FRC_INFO("[G1Sim2RealEnv.waitForLowState] Waiting to connect to the robot.");
    while (low_state_buffer_.GetCopy().tick() == 0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(control_dt_));
    }
    FRC_INFO("[G1Sim2RealEnv.waitForLowState] Successfully connected to the robot.");
}

void G1Sim2RealEnv::LowStateHandler(const void *message) {
  const LowState_& msg = *(const LowState_ *)message;
  // CRC verify
  if (msg.crc() != unitree_tools::Crc32Core((uint32_t *)&msg, (sizeof(LowState_) >> 2) - 1)) {
    FRC_ERROR("[G1Sim2RealEnv.LowStateHandler] CRC Error");
    return;
  }
  
  // 机器人类型模式更新
  if (mode_machine_ != msg.mode_machine()) { // 检查当前程序记录的机器人类型（mode_machine_）是否与最新状态中的不一致
    if (mode_machine_ == 0) FRC_INFO("[G1Sim2RealEnv.LowStateHandler] G1 type: " << unsigned(msg.mode_machine()));
    mode_machine_ = msg.mode_machine(); // 因为类型是 uint8_t，用 unsigned(...) 是为了防止 char 类型被当作 ASCII 打印。
  }
  
  // update gamepad
  // TODO: create a new thread to update gamepad in low freq e.g.-> policy_dt
  if (listenerPtr_) {
    memcpy(listenerPtr_->rx_.buff, &msg.wireless_remote()[0], 40);
    listenerPtr_->gamepad_.update(listenerPtr_->rx_.RF_RX);
    // FRC_INFO("[G1Sim2RealEnv.LowStateHandler] tick: "<<msg.tick()<<" Gamepad: lx=" << listenerPtr_->gamepad_.lx);
  }
  
  low_state_buffer_.SetData(msg);

  updateRobotState();
}

void G1Sim2RealEnv::updateRobotState() {
  LowState_ state = low_state_buffer_.GetCopy();

  // === 获取广义坐标 (generalized coordinates) ===
  Eigen::Vector3f rootPos = Eigen::Vector3f::Zero();  
  Eigen::Vector4f rootQuat;  
  rootQuat << state.imu_state().quaternion()[0],
              state.imu_state().quaternion()[1],
              state.imu_state().quaternion()[2],
              state.imu_state().quaternion()[3];
  Eigen::VectorXf jointPos(jointDim_);
  for (int i = 0; i < jointDim_; ++i) {
    jointPos[i] = state.motor_state()[i].q();
  }

  // === 获取速度项 ===
  Eigen::Vector3f rootVel = Eigen::Vector3f::Zero(); 
  Eigen::Vector3f rootAngVel;
  rootAngVel << state.imu_state().gyroscope()[0],
                state.imu_state().gyroscope()[1],
                state.imu_state().gyroscope()[2];
  Eigen::VectorXf jointVel(jointDim_);
  for (int i = 0; i < jointDim_; ++i) {
    jointVel[i] = state.motor_state()[i].dq();
  }
  
  if(cfg_->imu_type == "torso"){
    FRC_INFO("[G1Sim2RealEnv.updateRobotState] TODO: torso IMU type");
    throw std::runtime_error("torso IMU type is not supported!");
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

void G1Sim2RealEnv::sendCmd(LowCmd_& cmd) {
  cmd.crc() = unitree_tools::Crc32Core((uint32_t *)&cmd, (sizeof(cmd) >> 2) - 1);  // 计算 CRC 校验
  lowcmd_publisher_->Write(cmd);          // DDS 发布指令
}

void G1Sim2RealEnv::zeroTorqueState() {
  FRC_INFO("[G1Sim2RealEnv.zeroTorqueState] Enter zero torque state.");
  FRC_INFO("[G1Sim2RealEnv.zeroTorqueState] Waiting for the start signal...");
  Timer zeroTorqueStateTimer(control_dt_);

  while (listenerPtr_ && listenerPtr_->gamepad_.start.pressed != 1) {
    unitree_tools::create_zero_cmd(low_cmd_);  
    sendCmd(low_cmd_);        
    zeroTorqueStateTimer.wait();
  }
}

void G1Sim2RealEnv::moveToDefaultPos() {
  FRC_INFO("[G1Sim2RealEnv.moveToDefaultPos] Moving to default position...");
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
  LowState_ state = low_state_buffer_.GetCopy();
  for (int i = 0; i < dof_size; ++i) {
      init_dof_pos[i] = state.motor_state()[i].q();
  }

  // move to default pos
  for (int i = 0; i < num_step; ++i) {
      float alpha = static_cast<float>(i) / num_step;
      for (int j = 0; j < dof_size; ++j) {
          int motor_idx = j;
          float target_q = init_dof_pos[motor_idx] * (1.0f - alpha) + default_joint_pos[motor_idx] * alpha;
          low_cmd_.motor_cmd()[motor_idx].q() = target_q;
          low_cmd_.motor_cmd()[motor_idx].dq() = 0;
          low_cmd_.motor_cmd()[motor_idx].kp() = kps[motor_idx];
          low_cmd_.motor_cmd()[motor_idx].kd() = kds[motor_idx];
          low_cmd_.motor_cmd()[motor_idx].tau() = 0;
      }
      sendCmd(low_cmd_);
      moveToDefaultPosTimer.wait();
  }

  FRC_INFO("[G1Sim2RealEnv.moveToDefaultPos] Reached default position.");
}

void G1Sim2RealEnv::defaultPosState() {
  FRC_INFO("[G1Sim2RealEnv.defaultPosState] Enter default pos state.");
  FRC_INFO("[G1Sim2RealEnv.defaultPosState] Waiting for the Button A signal...");

  Timer defaultPosStateTimer(control_dt_);

  // config 参数
  const auto& kps = cfg_->kP;
  const auto& kds = cfg_->kD;
  const auto& default_joint_pos = cfg_->default_angles;
  int dof_size = jointDim_;

  while (listenerPtr_ && listenerPtr_->gamepad_.A.pressed != 1) {
    for (int i = 0; i < dof_size; ++i) {
      int motor_idx = i;
      low_cmd_.motor_cmd()[motor_idx].q() = default_joint_pos[motor_idx];
      low_cmd_.motor_cmd()[motor_idx].dq() = 0;
      low_cmd_.motor_cmd()[motor_idx].kp() = kps[motor_idx];
      low_cmd_.motor_cmd()[motor_idx].kd() = kds[motor_idx];
      low_cmd_.motor_cmd()[motor_idx].tau() = 0;
    }
    sendCmd(low_cmd_);
    defaultPosStateTimer.wait();
  }
}

void G1Sim2RealEnv::run() {
  // Timer controlTimer(control_dt_);
  RateLimiter controlTimer(1.0 / control_dt_, "real main loop");
  
  while (running_) {
    counter_++;

    // emergency stop button
    if (listenerPtr_ && listenerPtr_->gamepad_.select.pressed == 1) {
      FRC_INFO("[G1Sim2RealEnv.run] Emergency Stop! at " << counter_ << "count");
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
      FRC_INFO("[G1Sim2RealEnv.run] Waiting For actions!");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      actionPtr = jointCMDBufferPtr_->GetData();  // 重新尝试获取
    }

    {
      std::lock_guard<std::mutex> actionLock(action_lock_);
      const auto& cmd = *actionPtr;
      memcpy(pTarget.data(), cmd.data.position, sizeof(float) * jointDim_);
      memcpy(vTarget.data(), cmd.data.velocity, sizeof(float) * jointDim_);
      memcpy(jointPGain.data(), cmd.data.kp, sizeof(float) * jointDim_);
      memcpy(jointDGain.data(), cmd.data.kd, sizeof(float) * jointDim_);

      for (int i = 0; i < jointDim_; ++i) {
        low_cmd_.motor_cmd()[i].q()   = pTarget[i];
        low_cmd_.motor_cmd()[i].dq()  = vTarget[i];
        low_cmd_.motor_cmd()[i].kp()  = jointPGain[i];
        low_cmd_.motor_cmd()[i].kd()  = jointDGain[i];
        low_cmd_.motor_cmd()[i].tau() = 0.0f;
      }
    }

    sendCmd(low_cmd_);
    controlTimer.wait();
  }
}



