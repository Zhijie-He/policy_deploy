#include "state_machine/StateMachine.h"
#include "utility/timer.h"
#include "utility/logger.h"
#include "utility/orientation_tools.h"
#include "controller/UnitreePolicyWrapper.h"
#include "controller/EmanPolicyWrapper.h"
#include <thread>
#include <chrono>

StateMachine::StateMachine(std::shared_ptr<const BaseRobotConfig> cfg, const std::string& config_name)
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

  if (config_name == "g1_unitree") {
    _neuralCtrl = std::make_unique<UnitreePolicyWrapper>(cfg);
  } else if (config_name == "g1_eman") {
    _neuralCtrl = std::make_unique<EmanPolicyWrapper>(cfg);
  } else {
    throw std::runtime_error("Unsupported robot: " + config_name);
  }
}

void StateMachine::run(){
  Timer _loopTimer(_policyDt); // 创建一个定时器，周期是 _policyDt 秒（比如 0.01s，表示 100Hz）
  // while(_loopTimer.getMs() < 100) _loopTimer.wait(); // 等待系统至少运行 100ms 再开始主循环，常用于启动缓冲/初始化等待。  这个的延迟好像会导致policy的大幅变化
  while(_isRunning){
    
    auto t_start = std::chrono::high_resolution_clock::now();
    step();
    auto t_end = std::chrono::high_resolution_clock::now();
    
    double run_time_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    run_sum_us += run_time_us;
    run_sum_sq_us += run_time_us * run_time_us;
    ++run_count;

    if (run_count % 100 == 0) {
        double avg = run_sum_us / run_count;
        double stddev = std::sqrt(run_sum_sq_us / run_count - avg * avg);
        FRC_INFO("[StateMachine.run] Run AVG: " << avg << " us | STDDEV: " << stddev << " us");
        std::cout << "[StateMachine.run] Run AVG: " << avg << " us | STDDEV: " << stddev << " us\n";
         // 重置
        run_sum_us = 0;
        run_sum_sq_us = 0;
        run_count = 0;
    }

    _loopTimer.wait(); 
  }
}

void StateMachine::step(){
  parseRobotData();
  updateCommands();
  robotAction = _neuralCtrl->getControlAction(robotData);
  packJointAction();
  if (*_keyState != '\0') *_keyState = '\0';
}

void StateMachine::parseRobotData() {
  assert(_robotStatusBuffer != nullptr);
  auto status_ptr = _robotStatusBuffer->GetData();
  while (!status_ptr) { // 这里存在有可能jointCMD还没有设定值 但是这里在读取 所以要等待
    FRC_INFO("[StateMachine.parseRobotData] Waiting for robotStatusBuffer...");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    status_ptr = _robotStatusBuffer->GetData();  // 重新尝试获取
  }

  const auto& status = *status_ptr;  // 解引用拿结构体  推荐：零拷贝 + 明确只读引用
  robotData.timestamp = status.data.timestamp;

  robotData.basePosition = Eigen::Map<const Eigen::Vector3f>(status.data.position);
  robotData.baseQuat     = Eigen::Map<const Eigen::Vector4f>(status.data.position + 3);
  robotData.jointPosition = Eigen::Map<const Eigen::VectorXf>(status.data.position + 7, _jointNum);

  robotData.baseVelocity = Eigen::Map<const Eigen::Vector3f>(status.data.velocity);
  robotData.baseOmega    = Eigen::Map<const Eigen::Vector3f>(status.data.velocity + 3);
  robotData.jointVelocity = Eigen::Map<const Eigen::VectorXf>(status.data.velocity + 6, _jointNum);
}

void StateMachine::stop() { _isRunning = false; }

void StateMachine::updateCommands(){
  float deltaYaw = 0; // deltaYaw 表示当前目标朝向与机器人当前朝向的差值
  // Vec3f maxVelCmd{1.0, 0.3, 1.0}; // 最大线速度限制（1.0 前进，0.3 横向，0 竖直）；
  Vec3f maxVelCmd = cfg_->max_cmd;
  constexpr float maxYaw = 1.0f;
  
  // 键盘输入控制逻辑
  if (_keyState != nullptr && *_keyState != '\0') { 
    //  构造键盘增量指令
    Vec3<float> deltaVelTarg{0, 0, 0};
    Vec3<float> deltaAngTarg{0, 0, 0};

    constexpr float kStep = 0.05f;
    constexpr float kYawStep = 0.05f;
    constexpr float kThresh = 1e-2f;

    // 对每个按键设置线速度/角速度的增量。
    if (*_keyState == 'w') {
      deltaVelTarg << kStep, 0, 0;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'w' → Forward +" << kStep);
    }
    else if (*_keyState == 's') {
      deltaVelTarg << -kStep, 0, 0;
      FRC_INFO("[StateMachine.updateCommands] Pressed 's' → Backward -"  << kStep);
    }
    else if (*_keyState == 'a') {
      deltaVelTarg << 0, kStep, 0;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'a' → Left +"  << kStep);
    }
    else if (*_keyState == 'd') {
      deltaVelTarg << 0, -kStep, 0;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'd' → Right -"  << kStep);
    }
    else if (*_keyState == 'q') {
      deltaAngTarg << 0, 0, kYawStep;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'q' → Turn left +" << kYawStep << "rad");
    }
    else if (*_keyState == 'e') {
      deltaAngTarg << 0, 0, -kYawStep;
      FRC_INFO("[StateMachine.updateCommands] Pressed 'e' → Turn right -" << kYawStep << "rad");
    }

    // 线速度增量
    if (deltaVelTarg.norm() > kThresh) {
      Vec3f targLinVel_b = robotData.targetCMD + deltaVelTarg;
      robotData.targetCMD = targLinVel_b.cwiseMin(maxVelCmd).cwiseMax(-maxVelCmd);
      if (robotData.targetCMD.norm() < kThresh) robotData.targetCMD.setZero();  // 如果目标速度已经非常小，就设为 0； 起到“去抖动”的效果（防止机器人一直小幅抖动/晃动）。
      FRC_INFO("[StateMachine.updateCommands] Updated target velocity: " << robotData.targetCMD.transpose());
    }

    // 角速度目标（yawTarg）
    if (deltaAngTarg.norm() > kThresh) {  // .norm() 是什么？ 这是 Eigen 提供的函数，表示向量的 欧几里得范数（L2 范数），也就是： deltaAngTarg.norm() = sqrt(x² + y² + z²)
      // 在这个场景里，只有 z 分量会被赋值，所以其实就是：deltaAngTarg.norm() = abs(deltaAngTarg[2])
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

