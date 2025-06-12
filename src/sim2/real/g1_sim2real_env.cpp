// G1Sim2RealEnv.cpp
#include "sim2/real/g1_sim2real_env.h"
#include "utility/logger.h"
#include <thread>
#include "utility/timer.h"
#include "utility/real/unitree_tools.h"

void print_lowcmd(const LowCmd_& cmd) {
    std::cout << "=== LowCmd_ ===" << std::endl;
    std::cout << "Mode PR: " << (int)cmd.mode_pr() << std::endl;
    std::cout << "Mode Machine: " << (int)cmd.mode_machine() << std::endl;

    for (size_t i = 0; i < cmd.motor_cmd().size(); ++i) {
        const auto& m = cmd.motor_cmd().at(i);
        std::cout << "Motor[" << i << "]"
                  << "  mode=" << (int)m.mode()
                  << "  tau=" << m.tau()
                  << "  q=" << m.q()
                  << "  dq=" << m.dq()
                  << "  kp=" << m.kp()
                  << "  kd=" << m.kd()
                  << std::endl;
    }
}

void init_cmd_hg(LowCmd_& cmd, uint8_t mode_machine, Mode mode_pr) {
  cmd.mode_machine() = mode_machine;
  cmd.mode_pr() = static_cast<uint8_t>(mode_pr);
  
  // here should follow num actions
  size_t size = cmd.motor_cmd().size();
  for (size_t i = 0; i < size; ++i) {
      cmd.motor_cmd()[i].mode() = 1;
      cmd.motor_cmd()[i].q() = 0;
      cmd.motor_cmd()[i].dq() = 0;
      cmd.motor_cmd()[i].kp() = 0;
      cmd.motor_cmd()[i].kd() = 0;
      cmd.motor_cmd()[i].tau() = 0;
  }
}

void create_zero_cmd(LowCmd_& cmd) {
  size_t size = cmd.motor_cmd().size();
  for (size_t i = 0; i < size; ++i) {
    cmd.motor_cmd()[i].q() = 0.0f;
    cmd.motor_cmd()[i].dq() = 0.0f;
    cmd.motor_cmd()[i].kp() = 0.0f;
    cmd.motor_cmd()[i].kd() = 0.0f;
    cmd.motor_cmd()[i].tau() = 0.0f;
  }
}

G1Sim2RealEnv::G1Sim2RealEnv(const std::string& net_interface,
            std::shared_ptr<const BaseRobotConfig> cfg,
            const std::string& mode,
            const std::string& track,
            const std::vector<std::string>& track_list,
            std::shared_ptr<CustomTypes::MocapConfig> mocap_cfg,  
            std::shared_ptr<CustomTypes::VlaConfig> vla_cfg,
            std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr,
            std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr,
            std::shared_ptr<StateMachine> state_machine)
    : BaseEnv(cfg, jointCMDBufferPtr, robotStatusBufferPtr),
      net_interface_(net_interface),
      mode_pr_(Mode::PR),
      mode_machine_(0),
      mode_(mode),
      track_(track),
      track_list_(track_list),
      mocap_cfg_(mocap_cfg),
      vla_cfg_(vla_cfg),
      state_machine_(state_machine)
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
      FRC_ERROR("[G1Sim2RealEnv] Invalid msg_type" << cfg_->msg_type);
      throw std::invalid_argument("Invalid msg_type: " + cfg_->msg_type);
    }
    
    initState();
    
    // wait for the subscriber to receive data
    waitForLowState();

    // Initialize the command msg
    init_cmd_hg(low_cmd_, mode_machine_, mode_pr_);

    // print_lowcmd(low_cmd_);
}

G1Sim2RealEnv::G1Sim2RealEnv(const std::string& net_interface,
            std::shared_ptr<const BaseRobotConfig> cfg,
            const std::string& mode,
            const std::string& track,
            const std::vector<std::string>& track_list,
            std::shared_ptr<CustomTypes::MocapConfig> mocap_cfg,  
            std::shared_ptr<CustomTypes::VlaConfig> vla_cfg,
            std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr,
            std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr)
    : BaseEnv(cfg, jointCMDBufferPtr, robotStatusBufferPtr),
      net_interface_(net_interface),
      mode_pr_(Mode::PR),
      mode_machine_(0),
      mode_(mode),
      track_(track),
      track_list_(track_list),
      mocap_cfg_(mocap_cfg),
      vla_cfg_(vla_cfg)
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
      FRC_ERROR("[G1Sim2RealEnv] Invalid msg_type" << cfg_->msg_type);
      throw std::invalid_argument("Invalid msg_type: " + cfg_->msg_type);
    }
    
    initState();
    
    // wait for the subscriber to receive data
    waitForLowState();

    // Initialize the command msg
    init_cmd_hg(low_cmd_, mode_machine_, mode_pr_);

    // print_lowcmd(low_cmd_);
}

void G1Sim2RealEnv::initState() {
  gcDim_    = cfg_->num_actions + 7; 
  gvDim_    = cfg_->num_actions + 6; 
  jointDim_ = cfg_->num_actions; 

  // ① 机器人状态变量
  gc_.setZero(gcDim_);      //  当前 generalized coordinate（广义坐标，位置）
  gv_.setZero(gvDim_);      //  当前 generalized velocity（广义速度）

  // ② 控制增益
  jointPGain = cfg_->kP;
  jointDGain = cfg_->kD;

  // ③ 动作目标
  pTarget.setZero(jointDim_);
  pTarget = cfg_->default_angles;// desired position（用于位置控制）
  FRC_INFO("[G1Sim2RealEnv.initState] default_angles: " << cfg_->default_angles.transpose());
  vTarget.setZero(jointDim_); // desired velocity（用于速度控制）

  // ④ 力矩命令
  tauCmd.setZero(jointDim_); //	最终输出的控制力矩（全体）
}

void G1Sim2RealEnv::waitForLowState() {
    FRC_INFO("[G1Sim2RealEnv.waitForLowState] Waiting to connect to the robot.");
    while (low_state_.tick() == 0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(control_dt_));
    }
    FRC_INFO("[G1Sim2RealEnv.waitForLowState] Successfully connected to the robot.");
}

void G1Sim2RealEnv::LowStateHandler(const void *message) {
  low_state_ = *(const LowState_ *)message;
  if (low_state_.crc() != unitree_tools::Crc32Core((uint32_t *)&low_state_, (sizeof(LowState_) >> 2) - 1)) {
    FRC_ERROR("[G1Sim2RealEnv.LowStateHandler] CRC Error");
    return;
  }

  // 机器人类型模式更新
  if (mode_machine_ != low_state_.mode_machine()) { // 检查当前程序记录的机器人类型（mode_machine_）是否与最新状态中的不一致
    if (mode_machine_ == 0) FRC_INFO("[G1Sim2RealEnv.LowStateHandler] G1 type: " << unsigned(low_state_.mode_machine()));
    mode_machine_ = low_state_.mode_machine(); // 因为类型是 uint8_t，用 unsigned(...) 是为了防止 char 类型被当作 ASCII 打印。
  }

  if (listenerPtr_) {
    // update gamepad
    memcpy(listenerPtr_->rx_.buff, &low_state_.wireless_remote()[0], 40);
    listenerPtr_->gamepad_.update(listenerPtr_->rx_.RF_RX);
    FRC_INFO("[G1Sim2RealEnv.LowStateHandler] tick: "<<low_state_.tick()<<" Gamepad: lx=" << listenerPtr_->gamepad_.lx
            << " ly=" << listenerPtr_->gamepad_.ly
            << " rx=" << listenerPtr_->gamepad_.rx
            << " ry=" << listenerPtr_->gamepad_.ry
            << " | A=" << static_cast<int>(listenerPtr_->gamepad_.A.pressed)
            << " B=" << static_cast<int>(listenerPtr_->gamepad_.B.pressed)
            << " X=" << static_cast<int>(listenerPtr_->gamepad_.X.pressed)
            << " Y=" << static_cast<int>(listenerPtr_->gamepad_.Y.pressed)
            << " select=" << static_cast<int>(listenerPtr_->gamepad_.select.pressed)
            << " start=" << static_cast<int>(listenerPtr_->gamepad_.start.pressed));
  }
  
  updateRobotState();
}

void G1Sim2RealEnv::updateRobotState() {
  Eigen::VectorXf positionVec, velocityVec;
  float timestamp;

  {
    std::lock_guard<std::mutex> stateLock(state_lock_);

    // === 获取广义坐标 (generalized coordinates) ===
    Eigen::Vector3d basePos = Eigen::Vector3d::Zero();  // 暂设为 0
    Eigen::Vector4d baseQuat;  // 四元数
    baseQuat << low_state_.imu_state().quaternion()[0],
                low_state_.imu_state().quaternion()[1],
                low_state_.imu_state().quaternion()[2],
                low_state_.imu_state().quaternion()[3];

    Eigen::VectorXd jointPos(jointDim_);
    for (int i = 0; i < jointDim_; ++i) {
      jointPos[i] = low_state_.motor_state()[i].q();
    }

    gc_ << basePos, baseQuat, jointPos;

    // === 获取速度项 ===
    Eigen::Vector3d baseVelocity = Eigen::Vector3d::Zero();  // 暂设为 0
    Eigen::Vector3d baseOmega;
    baseOmega << low_state_.imu_state().gyroscope()[0],
                 low_state_.imu_state().gyroscope()[1],
                 low_state_.imu_state().gyroscope()[2];

    Eigen::VectorXd jointVel(jointDim_);
    for (int i = 0; i < jointDim_; ++i) {
      jointVel[i] = low_state_.motor_state()[i].dq();
    }

    gv_ << baseVelocity, baseOmega, jointVel;

    // === 转换为 float 用于 status 共享 ===
    positionVec = gc_.cast<float>();
    velocityVec = gv_.cast<float>();
    timestamp = low_state_.tick();  // 如果你用 mj_data_ 模拟器，也可 mj_data_->time
    FRC_INFO("[G1Sim2RealEnv.updateRobotState] gc_ dim = " << gc_.size() << ", values = " << gc_.transpose());
    // FRC_INFO("[G1Sim2RealEnv.updateRobotState] gv_ dim = " << gv_.size() << ", values = " << gv_.transpose());
    // FRC_INFO("[G1Sim2RealEnv.updateRobotState] timestamp/low_state_.tick() = " << timestamp);
  }

  // === 更新 robotStatus 共享结构 ===
  if (robotStatusBufferPtr_) {
    robotStatus status;
    memcpy(status.data.position, positionVec.data(), gcDim_ * sizeof(float));
    memcpy(status.data.velocity, velocityVec.data(), gvDim_ * sizeof(float));
    status.data.timestamp = timestamp;
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

  // while (listenerPtr_ && listenerPtr_->remote.button[KeyMap::start] != 1) {
  while (listenerPtr_ && listenerPtr_->gamepad_.start.pressed != 1) {
    create_zero_cmd(low_cmd_);  
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
  for (int i = 0; i < dof_size; ++i) {
      init_dof_pos[i] = low_state_.motor_state()[i].q();
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

  // while (remote_controller.button[KeyMap::A] != 1) {
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
  Timer controlTimer(control_dt_);

  while (running_) {
    counter_++;

    if(state_machine_) state_machine_->step();
    
    auto actionPtr = jointCMDBufferPtr_->GetData();  // 返回的是 std::shared_ptr<const jointCMD>
    while (!actionPtr) { 
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      actionPtr = jointCMDBufferPtr_->GetData();  // 重新尝试获取
    }

    auto action = std::make_shared<jointCMD>();
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

    if (listenerPtr_ && listenerPtr_->gamepad_.select.pressed == 1) {
      running_ = false;
    }

    controlTimer.wait();
  }
}



