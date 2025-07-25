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
  constexpr float kStep = 0.05f;
  constexpr float kYawStep = 0.05f;
  constexpr float kThresh = 1e-2f;
  constexpr float kDeadZone = 0.05f;

  // ==========================
  // 🎮 Joystick 控制逻辑
  // ==========================
  if (listenerPtr_ && listenerPtr_->gamepad_.isConnected()) {
    const auto joy = listenerPtr_->getJoystickState();

    // 对摇杆输入做 DeadZone 防抖（避免 0.001 等小输入抖动）
    float lx = (std::fabs(joy.lx) > kDeadZone) ? joy.lx : 0.0f;
    float ly = (std::fabs(joy.ly) > kDeadZone) ? joy.ly : 0.0f;
    float rx = (std::fabs(joy.rx) > kDeadZone) ? joy.rx : 0.0f;

    bool joyUsed = std::fabs(lx) > 0.0f || std::fabs(ly) > 0.0f || std::fabs(rx) > 0.0f;

    if (joyUsed) {
      Vec3f cmdVel;
      cmdVel[0] = -maxVelCmd[0] * ly;   // 前后
      cmdVel[1] = -maxVelCmd[1] * lx;   // 左右
      cmdVel[2] = -maxVelCmd[2] * rx;   // yaw（右摇杆）

      // 只有当有明显变化时才赋值
      if ((cmdVel - robotData.targetCMD).norm() > kThresh) {
        robotData.targetCMD = cmdVel;
        yawTarg = cmdVel[2];
        FRC_INFO("[Joy] targetCMD ← " << robotData.targetCMD.transpose());
      }
    } else {
      // 若当前 targetCMD 不为零，说明之前有操作，需要清零
      if (robotData.targetCMD.norm() > kThresh) {
        robotData.targetCMD.setZero();
        yawTarg = 0.f;
        FRC_INFO("[Joy] targetCMD cleared");
      }
    }

    // 手动清零（按 B 键）
    if (joy.B.on_press) {
      robotData.targetCMD.setZero();
      yawTarg = 0.f;
      FRC_INFO("[Joy] targetCMD manually cleared by B");
    }
  }

  // ==========================
  // ⌨️ 键盘控制逻辑（fallback）
  // ==========================
  if (listenerPtr_ && listenerPtr_->getKeyboardInput() != '\0') {
    Vec3<float> deltaVelTarg{0, 0, 0};
    Vec3<float> deltaAngTarg{0, 0, 0};
    char key = listenerPtr_->getKeyboardInput();

    // 对每个按键设置线速度/角速度的增量。
    if (key == 'w') {
      deltaVelTarg << kStep, 0, 0;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'w' → Forward +" << kStep);
    } else if (key == 's') {
      deltaVelTarg << -kStep, 0, 0;
      FRC_INFO("[StateMachine.updateCommands] Pressed 's' → Backward -"  << kStep);
    } else if (key == 'a') {
      deltaVelTarg << 0, kStep, 0;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'a' → Left +"  << kStep);
    } else if (key == 'd') {
      deltaVelTarg << 0, -kStep, 0;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'd' → Right -"  << kStep);
    } else if (key == 'q') {
      deltaAngTarg << 0, 0, kYawStep;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'q' → Turn left +" << kYawStep << "rad");
    } else if (key == 'e') {
      deltaAngTarg << 0, 0, -kYawStep;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'e' → Turn right -" << kYawStep << "rad");
    } else if (key == ' ') {
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


