// sim2/real/Sim2WlRobotEnv.cpp
#include <thread>
#include "sim2/real/sim2wlrobot_env.h"

void create_zero_cmd(MotorCmds& cmd) {
    size_t size = cmd.pos().size();
    for (size_t i = 0; i < size; ++i) {
        cmd.pos()[i] = 0.0f;
        cmd.w()[i] = 0.0f;
        cmd.kp()[i] = 0.0f;
        cmd.kd()[i] = 0.0f;
        cmd.t()[i] = 0.0f;
        cmd.mode()[i] = 10;
    }
}

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

// void Sim2WlRobotEnv::sendCmd(MotorCmds& cmd) {
//   lowcmd_publisher_->Write(cmd);        
// }

void Sim2WlRobotEnv::sendCmd(MotorCmds& cmd) {
    // Step 1: 提取 sim 顺序数据（从 cmd 中获取）
    std::array<float, 12> pos_sim = cmd.pos();
    std::array<float, 12> vel_sim = cmd.w();
    std::array<float, 12> kp_sim  = cmd.kp();
    std::array<float, 12> kd_sim  = cmd.kd();

    // Step 2: sim → real 顺序
    std::array<float, 12> pos_real{}, vel_real{}, kp_real{}, kd_real{};
    for (int real_idx = 0; real_idx < 12; ++real_idx) {
        int sim_idx = real2sim_dof_map_[real_idx];
        pos_real[real_idx] = pos_sim[sim_idx];
        vel_real[real_idx] = vel_sim[sim_idx];
        kp_real[real_idx]  = kp_sim[sim_idx];
        kd_real[real_idx]  = kd_sim[sim_idx];
    }

    // Step 3: 反校准（为真机使用）
    for (int i = 0; i < 12; ++i) {
        int leg = i / 3;
        int joint = i % 3;

        float sign = (joint == 0) ? abad_sign_[leg] :
                     (joint == 1) ? hip_sign_[leg] :
                                    knee_sign_[leg];
        float offset = zero_offset_[leg][joint];

        pos_real[i] = pos_real[i] * sign + offset;
        vel_real[i] = vel_real[i] * sign;
        // kp/kd 无需校准
    }

    // Step 4: 覆盖 cmd（确保发送的是 real 顺序 + 反校准后数据）
    cmd.pos(pos_real);
    cmd.w(vel_real);
    cmd.kp(kp_real);
    cmd.kd(kd_real);

    // Step 5: 发布
    lowcmd_publisher_->Write(cmd);
}

void Sim2WlRobotEnv::zeroTorqueState() {
  FRC_HIGHLIGHT("[Sim2WlRobotEnv.zeroTorqueState] Sending zero Cmd...");
  FRC_HIGHLIGHT("[Sim2WlRobotEnv.zeroTorqueState] Waiting for the start signal and then move to transfer position...");
  Timer zeroTorqueStateTimer(control_dt_);

  // while (listenerPtr_ && listenerPtr_->gamepad_.start.pressed != 1) {
  while (listenerPtr_ && *listenerPtr_->getKeyInputPtr() != 's') {
    create_zero_cmd(low_cmd_);  
    sendCmd(low_cmd_);        
    zeroTorqueStateTimer.wait();
  }
}

void Sim2WlRobotEnv::moveToTransferPos() {
  FRC_INFO("[Sim2WlRobotEnv.moveToTransferPos] Moving to transfer position...");
  Timer moveToTransferPosTimer(control_dt_);

  const float total_time = 3.0f;
  int num_step = static_cast<int>(total_time / control_dt_);

  // config 参数
  const auto& kps = cfg_->kP;
  const auto& kds = cfg_->kD;
  const float transfer_joint_pos[12] = {0., 1.2, -2.7, -0., 1.2, -2.7, 0., 2.2, -2.7, -0., 2.2, -2.7};
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
          float target_q = init_dof_pos[motor_idx] * (1.0f - alpha) + transfer_joint_pos[motor_idx] * alpha;
          low_cmd_.pos()[motor_idx] = target_q;
          low_cmd_.w()[motor_idx] = 0;
          low_cmd_.kp()[motor_idx] = kps[motor_idx];
          low_cmd_.kd()[motor_idx] = kds[motor_idx];
          low_cmd_.t()[motor_idx]= 0;
      }
      sendCmd(low_cmd_);
      moveToTransferPosTimer.wait();
  }
  FRC_INFO("[Sim2WlRobotEnv.moveToTransferPos] Reached transfer position.");
}

void Sim2WlRobotEnv::transferPosState() {
  FRC_HIGHLIGHT("[Sim2WlRobotEnv.transferPosState] Sending transfer position cmd...");
  FRC_HIGHLIGHT("[Sim2WlRobotEnv.transferPosState] Waiting for the Button A signal and then move to default position...");

  Timer transferPosStateTimer(control_dt_);

  // config 参数
  const auto& kps = cfg_->kP;
  const auto& kds = cfg_->kD;
  const float transfer_joint_pos[12] = {0., 1.2, -2.7, -0., 1.2, -2.7, 0., 2.2, -2.7, -0., 2.2, -2.7};
  int dof_size = jointDim_;

  // while (listenerPtr_ && listenerPtr_->gamepad_.A.pressed != 1) {
  while (listenerPtr_ && *listenerPtr_->getKeyInputPtr() != 'a') {
    for (int i = 0; i < dof_size; ++i) {
      int motor_idx = i;
      low_cmd_.pos()[motor_idx] = transfer_joint_pos[motor_idx];
      low_cmd_.w()[motor_idx] = 0;
      low_cmd_.kp()[motor_idx] = kps[motor_idx];
      low_cmd_.kd()[motor_idx] = kds[motor_idx];
      low_cmd_.t()[motor_idx]= 0;
    }
    sendCmd(low_cmd_);
    transferPosStateTimer.wait();
  }
}

void Sim2WlRobotEnv::moveToDefaultPos() {
  FRC_INFO("[Sim2WlRobotEnv.moveToDefaultPos] Moving to default position...");
  Timer moveToDefaultPosTimer(control_dt_);

  const float total_time = 3.0f;
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

  FRC_HIGHLIGHT("[Sim2WlRobotEnv.moveToDefaultPos] Reached default position.");
}

void Sim2WlRobotEnv::defaultPosState() {
  FRC_HIGHLIGHT("[Sim2WlRobotEnv.defaultPosState] Sending default pos cmd...");
  FRC_HIGHLIGHT("[Sim2WlRobotEnv.defaultPosState] Waiting for the Button A signal and then start the policy...");

  Timer defaultPosStateTimer(control_dt_);

  // config 参数
  const auto& kps = cfg_->kP;
  const auto& kds = cfg_->kD;
  const auto& default_joint_pos = cfg_->default_angles;
  int dof_size = jointDim_;

  // while (listenerPtr_ && listenerPtr_->gamepad_.A.pressed != 1) {
  while (listenerPtr_ && *listenerPtr_->getKeyInputPtr() != 'a') {
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

bool Sim2WlRobotEnv::isRunning() {
    // if (listenerPtr_ && listenerPtr_->gamepad_.select.pressed == 1) {
    if (listenerPtr_ && *listenerPtr_->getKeyInputPtr() == 'e') {
        FRC_INFO("[Sim2WlRobotEnv] Emergency Stop!");
        FRC_INFO("[Sim2WlRobotEnv.run] Emergency Stop! at " << run_count << " count!");

        MotorCmds terminateCmd;
        for (int i = 0; i < jointDim_; ++i) {
          int motor_idx = i;
          terminateCmd.pos()[motor_idx] = 0;
          terminateCmd.w()[motor_idx] = 0;
          terminateCmd.kp()[motor_idx] = 0;
          terminateCmd.kd()[motor_idx] = 2;
          terminateCmd.t()[motor_idx]= 0;
          terminateCmd.mode()[motor_idx] = 10;
        }

        for (int j = 0; j < 100; j++){
          sendCmd(terminateCmd);
          std::this_thread::sleep_for(std::chrono::duration<double>(control_dt_));
        }
        return false;
    }
    return running_;
}

void Sim2WlRobotEnv::applyAction(const jointCMD& cmd) {
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
  sendCmd(low_cmd_);
}

void Sim2WlRobotEnv::run() {
  runControlLoop();
}


