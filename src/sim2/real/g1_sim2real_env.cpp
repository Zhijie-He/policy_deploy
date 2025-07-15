// sim2/simulator/G1Sim2RealEnv.cpp
#include <thread>
#include "utility/logger.h"
#include "utility/timer.h"
#include "sim2/real/g1_sim2real_env.h"

#define MOTOR_MAX 7
#define SENSOR_MAX 9

class RIS_Mode {
public:
    RIS_Mode(uint8_t id = 0, uint8_t status = 0x01, uint8_t timeout = 0)
        : id_(id & 0x0F), status_(status & 0x07), timeout_(timeout & 0x01) {}

    uint8_t modeToUint8() const {
        uint8_t mode = 0;
        mode |= (id_ & 0x0F);
        mode |= (status_ & 0x07) << 4;
        mode |= (timeout_ & 0x01) << 7;
        return mode;
    }
private:
    uint8_t id_;
    uint8_t status_;
    uint8_t timeout_;
};

G1Sim2RealEnv::G1Sim2RealEnv(const std::string& net_interface,
                             std::shared_ptr<const BaseRobotConfig> cfg,
                             const std::string& hands_type,
                             std::shared_ptr<StateMachine> state_machine)
    : BaseEnv(cfg, hands_type, state_machine),
      net_interface_(net_interface)
{  
  // unitree_tools::print_lowcmd(low_cmd_);
  initWorld();
  waitForLowState();  // wait for the subscriber to receive robot state
  unitree_tools::init_cmd_hg(low_cmd_, mode_machine_, mode_pr_);   // Initialize the command msg
  FRC_INFO("[G1Sim2RealEnv.Const] G1Sim2RealEnv Ready.");
}

void G1Sim2RealEnv::initWorld() {
  BaseEnv::initWorld();
  
  if(cfg_->imu_type == "torso"){
    // h1 and h1_2 imu is on the torso
    FRC_INFO("[G1Sim2RealEnv.updateRobotState] TODO: torso IMU type");
    throw std::runtime_error("torso IMU type is not supported!");
  }
  
  FRC_INFO("[G1Sim2RealEnv.Const] net_interface: " << net_interface_);
  ChannelFactory::Instance()->Init(0);  //, net_interface.c_str());   // initialize DDS communication

  // add robot state and action pub / sub
  if(cfg_->msg_type == "hg"){
    // g1 and h1_2 use the hg msg type
    mode_pr_ = Mode::PR;
    mode_machine_ = 0;

    // create publisher
    lowcmd_publisher_ = std::make_unique<ChannelPublisher<LowCmd_>>(cfg_->lowcmd_topic);
    lowcmd_publisher_->InitChannel();
    
    // create subscriber
    lowstate_subscriber_ = std::make_unique<ChannelSubscriber<LowState_>>(cfg_->lowstate_topic);
    lowstate_subscriber_->InitChannel(
      [this](const void *message) {
        this->LowStateHgHandler(message);
      }, 10
    );
  } else {
    FRC_ERROR("[G1Sim2RealEnv.Const] Invalid msg_type" << cfg_->msg_type);
    throw std::invalid_argument("Invalid msg_type: " + cfg_->msg_type);
  }
  
  // add left and right hands publisher
  if(hands_type_ == "dex3"){
    leftHandcmd_publisher_ = std::make_unique<ChannelPublisher<HandCmd_>>(cfg_->hand_map.at(hands_type_).left_cmd_topic);
    leftHandcmd_publisher_->InitChannel();
    
    rightHandcmd_publisher_ = std::make_unique<ChannelPublisher<HandCmd_>>(cfg_->hand_map.at(hands_type_).right_cmd_topic);
    rightHandcmd_publisher_->InitChannel();

    leftHand_state_.motor_state().resize(MOTOR_MAX);
    leftHand_state_.press_sensor_state().resize(SENSOR_MAX);
    leftHand_cmd_.motor_cmd().resize(MOTOR_MAX);

    rightHand_state_.motor_state().resize(MOTOR_MAX);
    rightHand_state_.press_sensor_state().resize(SENSOR_MAX);
    rightHand_cmd_.motor_cmd().resize(MOTOR_MAX);
  }

  // visualization
  {
    dds_entity_t dds_participant_ = dds_create_participant(0, NULL, NULL);
    if (dds_participant_ < 0) {
        FRC_ERROR("Failed to create DDS participant");
    }

    dds_entity_t dds_topic_ = dds_create_topic(dds_participant_, &mujoco_visualization_KeypointsMsg_desc, cfg_->mujoco_keypoints_topic.c_str(), NULL, NULL);
    if (dds_topic_ < 0) {
        FRC_ERROR("Failed to create DDS topic");
    }

    keypoints_publisher_ = dds_create_writer(dds_participant_, dds_topic_, NULL, NULL);
    if (keypoints_publisher_ < 0) {
        FRC_ERROR("Failed to create DDS writer");
    }
  }
}

void G1Sim2RealEnv::waitForLowState() {
  FRC_INFO("[G1Sim2RealEnv.waitForLowState] Waiting to connect to the robot.");
  while (low_state_buffer_.GetCopy().tick() == 0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(control_dt_));
  }
  FRC_INFO("[G1Sim2RealEnv.waitForLowState] Successfully connected to the robot.");
}

void G1Sim2RealEnv::LowStateHgHandler(const void *message) {
  const LowState_& msg = *(const LowState_ *)message;
  // CRC verify
  // if (msg.crc() != unitree_tools::Crc32Core((uint32_t *)&msg, (sizeof(LowState_) >> 2) - 1)) {
  //   FRC_ERROR("[G1Sim2RealEnv.LowStateHgHandler] CRC Error");
  //   return;
  // }
  
  // 机器人类型模式更新
  if (mode_machine_ != msg.mode_machine()) { // 检查当前程序记录的机器人类型（mode_machine_）是否与最新状态中的不一致
    if (mode_machine_ == 0) FRC_INFO("[G1Sim2RealEnv.LowStateHgHandler] G1 type: " << unsigned(msg.mode_machine()));
    mode_machine_ = msg.mode_machine(); // 因为类型是 uint8_t，用 unsigned(...) 是为了防止 char 类型被当作 ASCII 打印。
  }
  
  // update gamepad
  // TODO: create a new thread to update gamepad in low freq e.g.-> policy_dt
  if (listenerPtr_) {
    memcpy(listenerPtr_->rx_.buff, &msg.wireless_remote()[0], 40);
    listenerPtr_->gamepad_.update(listenerPtr_->rx_.RF_RX);
    // FRC_INFO("[G1Sim2RealEnv.LowStateHgHandler] tick: "<<msg.tick()<<" Gamepad: lx=" << listenerPtr_->gamepad_.lx);
  }
  
  low_state_buffer_.SetData(msg);
  updateRobotState();
}

void G1Sim2RealEnv::updateRobotState() {
  LowState_ state = low_state_buffer_.GetCopy();

  // === 获取广义坐标 (generalized coordinates) ===
  Eigen::Vector3f rootPos = Eigen::Vector3f::Zero();  // currently set 0
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
  Eigen::Vector3f rootVel = Eigen::Vector3f::Zero();  // currently set 0
  Eigen::Vector3f rootAngVel;
  rootAngVel << state.imu_state().gyroscope()[0],
                state.imu_state().gyroscope()[1],
                state.imu_state().gyroscope()[2];
  Eigen::VectorXf jointVel(jointDim_);
  for (int i = 0; i < jointDim_; ++i) {
    jointVel[i] = state.motor_state()[i].dq();
  }

  // === 填入 Root Pose ===
  gc_.segment(0, 3) = rootPos;
  gc_.segment(3, 4) = rootQuat;
  // === 填入 Root Velocity ===
  gv_.segment(0, 3) = rootVel;
  gv_.segment(3, 3) = rootAngVel;
  // === 手部状态：默认 0 ===
  Eigen::VectorXf handsJointPos = Eigen::VectorXf::Zero(handsDim_);
  Eigen::VectorXf handsJointVel = Eigen::VectorXf::Zero(handsDim_);

  // 构造 full_position 和 full_velocity（joint + hands）
  Eigen::VectorXf full_position = tools::concatJointAndHand(jointPos, handsJointPos, jointDim_, handsDim_);
  Eigen::VectorXf full_velocity = tools::concatJointAndHand(jointVel, handsJointVel, jointDim_, handsDim_);

  if (handsDim_ > 0) { // 如果需要兼容手的 reordering
    gc_.tail(actuatorDim_) = tools::resolveCompatibilityConcat(full_position, cfg_->hand_map.at(hands_type_).joint_concat_index);
    gv_.tail(actuatorDim_) = tools::resolveCompatibilityConcat(full_velocity, cfg_->hand_map.at(hands_type_).joint_concat_index);
  } else {
    gc_.tail(actuatorDim_) = full_position;
    gv_.tail(actuatorDim_) = full_velocity;
  }

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

  // while (listenerPtr_ && listenerPtr_->gamepad_.start.pressed != 1) {
  while (listenerPtr_ && *listenerPtr_->getKeyInputPtr() != 's') {
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
        low_cmd_.motor_cmd()[motor_idx].kp() = jointPGain[motor_idx];
        low_cmd_.motor_cmd()[motor_idx].kd() = jointDGain[motor_idx];
        low_cmd_.motor_cmd()[motor_idx].tau() = 0;
    }
    sendCmd(low_cmd_);
    moveToDefaultPosTimer.wait();
  }

  if (hands_type_ == "dex3") {
    for (int i = 0; i < MOTOR_MAX; ++i) {
      // Left hand
      {
        RIS_Mode ris_mode(i, 0x01, 0);
        uint8_t motor_mode = ris_mode.modeToUint8();
        leftHand_cmd_.motor_cmd()[i].mode(motor_mode);
        leftHand_cmd_.motor_cmd()[i].q(0);
        leftHand_cmd_.motor_cmd()[i].dq(0);
        leftHand_cmd_.motor_cmd()[i].kp(leftHandPGain[i]);
        leftHand_cmd_.motor_cmd()[i].kd(leftHandDGain[i]);
        leftHand_cmd_.motor_cmd()[i].tau(0);
      }

      // Right hand
      {
        RIS_Mode ris_mode(i, 0x01, 0);
        uint8_t motor_mode = ris_mode.modeToUint8();
        rightHand_cmd_.motor_cmd()[i].mode(motor_mode);
        rightHand_cmd_.motor_cmd()[i].q(0);
        rightHand_cmd_.motor_cmd()[i].dq(0);
        rightHand_cmd_.motor_cmd()[i].kp(rightHandPGain[i]);
        rightHand_cmd_.motor_cmd()[i].kd(rightHandDGain[i]);
        rightHand_cmd_.motor_cmd()[i].tau(0);
      }
    }
    leftHandcmd_publisher_->Write(leftHand_cmd_);
    rightHandcmd_publisher_->Write(rightHand_cmd_);
  }

  FRC_INFO("[G1Sim2RealEnv.moveToDefaultPos] Reached default position.");
}

void G1Sim2RealEnv::defaultPosState() {
  FRC_INFO("[G1Sim2RealEnv.defaultPosState] Enter default pos state.");
  FRC_INFO("[G1Sim2RealEnv.defaultPosState] Waiting for the Button A signal...");

  Timer defaultPosStateTimer(control_dt_);

  // config 参数
  const auto& default_joint_pos = cfg_->default_angles;
  int dof_size = jointDim_;

  // while (listenerPtr_ && listenerPtr_->gamepad_.A.pressed != 1) {
  while (listenerPtr_ && *listenerPtr_->getKeyInputPtr() != 'a') {
    for (int i = 0; i < dof_size; ++i) {
      int motor_idx = i;
      low_cmd_.motor_cmd()[motor_idx].q() = default_joint_pos[motor_idx];
      low_cmd_.motor_cmd()[motor_idx].dq() = 0;
      low_cmd_.motor_cmd()[motor_idx].kp() = jointPGain[motor_idx];
      low_cmd_.motor_cmd()[motor_idx].kd() = jointDGain[motor_idx];
      low_cmd_.motor_cmd()[motor_idx].tau() = 0;
    }
    sendCmd(low_cmd_);
    defaultPosStateTimer.wait();
  }
}

bool G1Sim2RealEnv::isRunning() const {
    if (listenerPtr_ && listenerPtr_->gamepad_.select.pressed == 1) {
        FRC_INFO("[G1Sim2RealEnv] Emergency Stop!");
        FRC_INFO("[G1Sim2RealEnv.run] Emergency Stop! at " << run_count << "count");
        return false;
    }
    return running_;
}

void G1Sim2RealEnv::applyAction(const jointCMD& cmd) {
  // 拆分（resolveCompatibilitySplit）
  Eigen::VectorXf position_vec = Eigen::Map<const Eigen::VectorXf>(cmd.data.position, actuatorDim_);
  auto [joint_pos, hands_joint_pos] = tools::resolveCompatibilitySplit(position_vec, cfg_->hand_map.at(hands_type_).joint_split_index);

  Eigen::VectorXf vel_vec = Eigen::Map<const Eigen::VectorXf>(cmd.data.velocity, actuatorDim_);
  auto [joint_vel, hands_joint_vel] = tools::resolveCompatibilitySplit(vel_vec, cfg_->hand_map.at(hands_type_).joint_split_index);
  
  for (int i = 0; i < jointDim_; ++i) {
    low_cmd_.motor_cmd()[i].q()   = joint_pos[i];
    low_cmd_.motor_cmd()[i].dq()  = joint_vel[i];
    low_cmd_.motor_cmd()[i].kp()  = jointPGain[i];
    low_cmd_.motor_cmd()[i].kd()  = jointDGain[i];
    low_cmd_.motor_cmd()[i].tau() = 0.0f;
  }
  sendCmd(low_cmd_);

  // set URDF Limits
  // const float maxLimits_left[7]=  {  1.05 ,  1.05  , 1.75 ,   0   ,  0    , 0     , 0   }; // set max motor value
  // const float minLimits_left[7]=  { -1.05 , -0.724 ,   0  , -1.57 , -1.75 , -1.57  ,-1.75}; 
  // const float maxLimits_right[7]= {  1.05 , 0.742  ,   0  ,  1.57 , 1.75  , 1.57  , 1.75}; 
  // const float minLimits_right[7]= { -1.05 , -1.05  , -1.75,    0  ,  0    ,   0   ,0    }; 

  if (hands_type_ == "dex3") {
    Eigen::VectorXf lefthand_pos  = hands_joint_pos.segment(0, 7);
    Eigen::VectorXf righthand_pos = hands_joint_pos.segment(7, 7);
    Eigen::VectorXf lefthand_vel  = hands_joint_vel.segment(0, 7);
    Eigen::VectorXf righthand_vel = hands_joint_vel.segment(7, 7);

    for (int i = 0; i < MOTOR_MAX; ++i) {
      // Left hand
      {
        RIS_Mode ris_mode(i, 0x01, 0);
        uint8_t motor_mode = ris_mode.modeToUint8();
        leftHand_cmd_.motor_cmd()[i].mode(motor_mode);
        leftHand_cmd_.motor_cmd()[i].q(lefthand_pos[i]);
        leftHand_cmd_.motor_cmd()[i].dq(lefthand_vel[i]);
        leftHand_cmd_.motor_cmd()[i].kp(leftHandPGain[i]);
        leftHand_cmd_.motor_cmd()[i].kd(leftHandDGain[i]);
        leftHand_cmd_.motor_cmd()[i].tau(0);
      }

      // Right hand
      {
        RIS_Mode ris_mode(i, 0x01, 0);
        uint8_t motor_mode = ris_mode.modeToUint8();
        rightHand_cmd_.motor_cmd()[i].mode(motor_mode);
        rightHand_cmd_.motor_cmd()[i].q(righthand_pos[i]);
        rightHand_cmd_.motor_cmd()[i].dq(righthand_vel[i]);
        rightHand_cmd_.motor_cmd()[i].kp(rightHandPGain[i]);
        rightHand_cmd_.motor_cmd()[i].kd(rightHandDGain[i]);
        rightHand_cmd_.motor_cmd()[i].tau(0);
      }
    }

    leftHandcmd_publisher_->Write(leftHand_cmd_);
    rightHandcmd_publisher_->Write(rightHand_cmd_);
  }

  // 获取可视化数据
  auto visualization = state_machine_->getTaskVisualization();
  if (visualization.rows() > 0) {
    keypoints_msg_ = {0};
    for (int i = 0; i < 90; ++i) keypoints_msg_.keypoints[i] = 0.0f;
    int idx = 0;
    for (int i = 0; i < visualization.rows(); ++i) {
        for (int j = 0; j < visualization.cols(); ++j) {
            if (idx >= 90) break;
            keypoints_msg_.keypoints[idx++] = visualization(i, j);
        }
    }
    // 发布消息
    int rc = dds_write(keypoints_publisher_, &keypoints_msg_);
    if (rc < 0) {
        FRC_WARN("Failed to write DDS visualization message");
    } else {
      FRC_HIGHLIGHT("publised visualization msessge");
    }
  }
}

void G1Sim2RealEnv::run() {
  runControlLoop();
}
