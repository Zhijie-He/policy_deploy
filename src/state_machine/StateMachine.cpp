#include "state_machine/StateMachine.h"
#include "utility/timer.h"
#include "utility/tools.h"
#include "utility/logger.h"
#include "policy_wrapper/UnitreePolicyWrapper.h"
#include "policy_wrapper/EmanPolicyWrapper.h"
#include <thread>
#include <chrono>

StateMachine::StateMachine(std::shared_ptr<const BaseRobotConfig> cfg, 
                           const std::string& config_name, 
                           torch::Device device,
                           const std::string& inference_engine_type,
                           const std::string& precision)
    : cfg_(cfg)
{
  // 1. 初始化底层设备数据结构
  _robotStatusBuffer = std::make_shared<DataBuffer<robotStatus>>();
  _jointCMDBuffer = std::make_shared<DataBuffer<jointCMD>>();
    
  // 2. 读取控制周期和关节数
  _policyDt = cfg_->getPolicyDt();
  _jointNum = cfg->num_actions;

  // 3. 初始化机器人状态和输出动作向量为零
  robotAction = CustomTypes::zeroAction(_jointNum);
  robotData = CustomTypes::zeroData(_jointNum);
    
  if (!cfg_->cmd_init.isZero()) {
    robotData.targetCMD = cfg_->cmd_init;
    FRC_INFO("[StateMachine.Const] Initial target cmd: " << robotData.targetCMD.transpose()); 
  } 
  
  // 4. get control policy
  _neuralCtrl = tools::loadPolicyWrapper(config_name, cfg, device, inference_engine_type, precision);
}

void StateMachine::step(){
  getRawObs();
  updateCommands();
  robotAction = _neuralCtrl->getControlAction(robotData);
  packJointAction();
  if (listenerPtr_ && listenerPtr_->getKeyboardInput() != '\0') listenerPtr_->clearKeyboardInput();
}

void StateMachine::getRawObs() {
  assert(_robotStatusBuffer != nullptr);
  auto status_ptr = _robotStatusBuffer->GetData();
  while (!status_ptr) { // 这里存在有可能jointCMD还没有设定值 但是这里在读取 所以要等待
    FRC_INFO("[StateMachine.getRawObs] Waiting for robotStatusBuffer...");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    status_ptr = _robotStatusBuffer->GetData();  // 重新尝试获取
  }

  const auto& status = *status_ptr;  // 解引用拿结构体  推荐：零拷贝 + 明确只读引用
  robotData.timestamp = status.data.timestamp;

  robotData.root_xyz = Eigen::Map<const Eigen::Vector3f>(status.data.position);
  robotData.root_rot     = Eigen::Map<const Eigen::Vector4f>(status.data.position + 3);
  robotData.joint_pos = Eigen::Map<const Eigen::VectorXf>(status.data.position + 7, _jointNum);

  robotData.root_vel = Eigen::Map<const Eigen::Vector3f>(status.data.velocity);
  robotData.root_ang_vel    = Eigen::Map<const Eigen::Vector3f>(status.data.velocity + 3);
  robotData.joint_vel = Eigen::Map<const Eigen::VectorXf>(status.data.velocity + 6, _jointNum);

  robotData.joint_torques = Eigen::Map<const Eigen::VectorXf>(status.data.jointTorques, _jointNum);
}

void StateMachine::stop() { _isRunning = false; }

void StateMachine::updateCommands(){
  float deltaYaw = 0; // deltaYaw 表示当前目标朝向与机器人当前朝向的差值
  Vec3f maxVelCmd = cfg_->max_cmd;
  const float maxYaw = maxVelCmd[2];

  // 键盘输入控制逻辑
  if (listenerPtr_ != nullptr && listenerPtr_->getKeyboardInput() != '\0') {
    //  构造键盘增量指令
    Vec3<float> deltaVelTarg{0, 0, 0};
    Vec3<float> deltaAngTarg{0, 0, 0};

    constexpr float kStep = 0.05f;
    constexpr float kYawStep = 0.05f;
    constexpr float kThresh = 1e-2f;

    // 对每个按键设置线速度/角速度的增量。
    if (listenerPtr_->getKeyboardInput() == 'w') {
      deltaVelTarg << kStep, 0, 0;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'w' → Forward +" << kStep);
    } else if (listenerPtr_->getKeyboardInput() == 's') {
      deltaVelTarg << -kStep, 0, 0;
      FRC_INFO("[StateMachine.updateCommands] Pressed 's' → Backward -"  << kStep);
    } else if (listenerPtr_->getKeyboardInput() == 'a') {
      deltaVelTarg << 0, kStep, 0;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'a' → Left +"  << kStep);
    } else if (listenerPtr_->getKeyboardInput() == 'd') {
      deltaVelTarg << 0, -kStep, 0;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'd' → Right -"  << kStep);
    } else if (listenerPtr_->getKeyboardInput() == 'q') {
      deltaAngTarg << 0, 0, kYawStep;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'q' → Turn left +" << kYawStep << "rad");
    } else if (listenerPtr_->getKeyboardInput() == 'e') {
      deltaAngTarg << 0, 0, -kYawStep;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'e' → Turn right -" << kYawStep << "rad");
    } else if (listenerPtr_->getKeyboardInput() == ' ') {
      robotData.targetCMD.setZero();
      yawTarg = 0.f;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'space' → Reset target velocity and yaw to zero.");
    }

    // 线速度增量
    if (deltaVelTarg.norm() > kThresh) {
      Vec3f targLinVel_b = robotData.targetCMD + deltaVelTarg;
      robotData.targetCMD = targLinVel_b.cwiseMin(maxVelCmd).cwiseMax(-maxVelCmd);
      if (robotData.targetCMD.norm() < kThresh) robotData.targetCMD.setZero();  // 如果目标速度已经非常小，就设为 0； 起到“去抖动”的效果（防止机器人一直小幅抖动/晃动）。
      FRC_INFO("[StateMachine.updateCommands] Updated target velocity: " << robotData.targetCMD.transpose());
    }

    // 角速度目标（yawTarg）
    if (deltaAngTarg.norm() > kThresh) { 
      float targYaw = robotData.targetCMD[2] + deltaAngTarg[2];  // 增量叠加
      targYaw = fmod(targYaw + static_cast<float>(M_PI), 2.0f * static_cast<float>(M_PI)) - static_cast<float>(M_PI); // wrap
      targYaw = std::clamp(targYaw, -maxYaw, maxYaw);  // 限幅
      if (std::abs(targYaw) < kThresh) targYaw = 0.f;  // 去抖动
      robotData.targetCMD[2] = targYaw;
      yawTarg = targYaw;  // 同步更新 yawTarg（如果你还要记录它）
      FRC_INFO("[StateMachine.updateCommands] Updated target yaw: " << robotData.targetCMD.transpose());
    }
  }
}

void StateMachine::packJointAction(){
  assert(_jointCMDBuffer != nullptr);
  jointCMD cmd;
  cmd.data.timestamp = robotAction.timestamp;
  memcpy(cmd.data.position, robotAction.motorPosition.data(), _jointNum * sizeof(float));
  memcpy(cmd.data.velocity, robotAction.motorVelocity.data(), _jointNum * sizeof(float));
  memcpy(cmd.data.kp, robotAction.kP.data(), _jointNum * sizeof(float));
  memcpy(cmd.data.kd, robotAction.kD.data(), _jointNum * sizeof(float));
  _jointCMDBuffer->SetData(cmd);
}


