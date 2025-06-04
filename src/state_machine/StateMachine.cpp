#include "state_machine/StateMachine.h"
#include "utility/timer.h"
#include "utility/logger.h"
#include "utility/orientation_tools.h"

StateMachine::StateMachine(std::shared_ptr<const BaseRobotConfig> cfg)
    : cfg_(cfg)
{
  // 1. 初始化底层设备数据结构
  _robotStatus = new robotStatus();
  _jointCMD = new jointCMD();
  
  // 2. 读取控制周期和关节数
  _policyDt = cfg_->getPolicyDt();
  _jointNum = cfg->num_actions;

  // 3. 初始化机器人状态和输出动作向量为零
  robotAction = CustomTypes::zeroAction(_jointNum);
  robotData = CustomTypes::zeroData(_jointNum);
    
  if (!cfg_->cmd_init.isZero()) {
      robotData.targetCMD = cfg_->cmd_init;
      FRC_INFO("Initial target cmd: " << robotData.targetCMD.transpose()); 
  }
}

void StateMachine::run(){
  Timer _loopTimer(_policyDt); // 创建一个定时器，周期是 _policyDt 秒（比如 0.01s，表示 100Hz）
  // while(_loopTimer.getMs() < 100) _loopTimer.wait(); // 等待系统至少运行 100ms 再开始主循环，常用于启动缓冲/初始化等待。  这个的延迟好像会导致policy的大幅变化
  while(_isRunning){
    step();
    _loopTimer.wait(); 
  }
}

void StateMachine::parseRobotData() {
  assert(_robotStatus != nullptr);

  robotData.timestamp = _robotStatus->data.timestamp;

  robotData.basePosition = Eigen::Map<Eigen::Vector3f>(_robotStatus->data.position);
  robotData.baseQuat     = Eigen::Map<Eigen::Vector4f>(_robotStatus->data.position + 3);
  robotData.jointPosition = Eigen::Map<Eigen::VectorXf>(_robotStatus->data.position + 7, _jointNum);

  robotData.baseVelocity = Eigen::Map<Eigen::Vector3f>(_robotStatus->data.velocity);
  robotData.baseOmega    = Eigen::Map<Eigen::Vector3f>(_robotStatus->data.velocity + 3);
  robotData.jointVelocity = Eigen::Map<Eigen::VectorXf>(_robotStatus->data.velocity + 6, _jointNum);
}


void StateMachine::stop() { _isRunning = false; }


void StateMachine::updateCommands(){
  float deltaYaw = 0; // deltaYaw 表示当前目标朝向与机器人当前朝向的差值
  Vec3f maxVelCmd{1.0, 0.3,0}; // 最大线速度限制（1.0 前进，0.3 横向，0 竖直）；

  /// keyboard input 2️⃣ 🔤 键盘输入控制逻辑
  if (_keyState != nullptr && *_keyState != '\0') { // ✅ 有有效的键盘输入才继续处理。
    //  构造键盘增量指令
    Vec3<float> deltaVelTarg{0, 0, 0};
    Vec3<float> deltaAngTarg{0, 0, 0};
    // ✅ 对每个按键设置线速度/角速度的增量。
    if (*_keyState == 'w') {
      deltaVelTarg << 0.1, 0, 0;
      FRC_INFO("[KEYBOARD] Pressed 'w' → Forward +0.1");
    }
    else if (*_keyState == 's') {
      deltaVelTarg << -0.1, 0, 0;
      FRC_INFO("[KEYBOARD] Pressed 's' → Backward -0.1");
    }
    else if (*_keyState == 'a') {
      deltaVelTarg << 0, 0.1, 0;
      FRC_INFO("[KEYBOARD] Pressed 'a' → Left +0.1");
    }
    else if (*_keyState == 'd') {
      deltaVelTarg << 0, -0.1, 0;
      FRC_INFO("[KEYBOARD] Pressed 'd' → Right -0.1");
    }
    else if (*_keyState == 'q') {
      deltaAngTarg << 0, 0, 0.1;
      FRC_INFO("[KEYBOARD] Pressed 'q' → Turn left +0.1 rad");
    }
    else if (*_keyState == 'e') {
      deltaAngTarg << 0, 0, -0.1;
      FRC_INFO("[KEYBOARD] Pressed 'e' → Turn right -0.1 rad");
    }

    // 应用线速度增量
    if (deltaVelTarg.norm() > 1e-2) {
      // Vec3f targLinVel_b = targLinVel_b + deltaVelTarg;
      Vec3f targLinVel_b = robotData.targetCMD + deltaVelTarg;
      robotData.targetCMD = targLinVel_b
                                  .cwiseMin(maxVelCmd)
                                  .cwiseMax(-maxVelCmd);
      // 逐个分量限制目标速度在 [-maxVelCmd, +maxVelCmd] 范围内；
      // 防止用户按太多次键盘速度无限加上去。

      // 如果目标速度已经非常小，就设为 0； 起到“去抖动”的效果（防止机器人一直小幅抖动/晃动）。
      if (robotData.targetCMD.norm() < 1e-2) robotData.targetCMD.setZero();
       // ✅ 输出当前目标速度
      FRC_INFO("[KEYBOARD] Updated target velocity: "
              << robotData.targetCMD.transpose());
    }

    // 应用角速度目标（yawTarg）
    if (deltaAngTarg.norm() > 1e-2) { // 判断角速度增量 deltaAngTarg 是否足够大（是否显著）来触发 yaw 控制更新。
      // .norm() 是什么？ 这是 Eigen 提供的函数，表示向量的 欧几里得范数（L2 范数），也就是： deltaAngTarg.norm() = sqrt(x² + y² + z²)
      // 在这个场景里，只有 z 分量会被赋值，所以其实就是：deltaAngTarg.norm() = abs(deltaAngTarg[2])
      // > 1e-2 是什么意思？ 这是一个 阈值判断，即： if (abs(deltaAngTarg[2]) > 0.01) 意思是只有当角速度增量绝对值超过 0.01 rad/s（大概 ≈ 0.57°/s）才认为是有效输入，才执行以下逻辑。
      yawTarg = yawTarg + deltaAngTarg[2];
      // yawTarg = fmod(yawTarg + M_PIf32, 2 * M_PIf32) - M_PIf32;
      yawTarg = fmod(yawTarg + static_cast<float>(M_PI), 2 * static_cast<float>(M_PI)) - static_cast<float>(M_PI);
      FRC_INFO("[KEYBOARD] Updated yawTarg: " << yawTarg);
    }

    // ✅ 将 yawTarg 夹到 [-π, π] 的范围内（wrap 到标准角度区间）
    // 角度是周期变量，比如 370° 和 10° 在方向上是一样的。 所以为了保持 yaw 表达的一致性，我们总是希望它在 -π 到 π（-180° 到 180°）之间。
    if (yawTarg > M_PI) yawTarg -= 2 * M_PI;
    else if (yawTarg < -M_PI) yawTarg += 2 * M_PI;
  }
}

void StateMachine::packJointAction(){
  assert(_jointCMD != nullptr);
  _jointCMD->data.timestamp = robotAction.timestamp;
  memcpy(_jointCMD->data.position, robotAction.motorPosition.data(), _jointNum * sizeof(float));
  memcpy(_jointCMD->data.velocity, robotAction.motorVelocity.data(), _jointNum * sizeof(float));
  memcpy(_jointCMD->data.kp, robotAction.kP.data(), _jointNum * sizeof(float));
  memcpy(_jointCMD->data.kd, robotAction.kD.data(), _jointNum * sizeof(float));
}