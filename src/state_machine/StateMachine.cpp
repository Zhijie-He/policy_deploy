#include "state_machine/StateMachine.h"
#include "utility/timer.h"
#include "utility/logger.h"
#include "utility/orientation_tools.h"

StateMachine::StateMachine(std::shared_ptr<const RobotConfig> cfg)
    : cfg_(cfg)
{
  // 1. 初始化底层设备数据结构（jointTarget, jointState, Gyro）
  _motorTargets = new jointTargetData(); //	控制输出（目标位置/速度/增益）结构体
  _motorStates = new jointStateData (); // 	控制输入（实际状态）结构体
  _gyroStates = new GyroData(); // 	IMU 数据
  
  // 2. 读取控制周期和关节数
  _policyDt = cfg_->getPolicyDt(); // 控制周期、关节数，从 YAML 读取
  _jointNum = cfg->num_actions;

  // 3. 初始化机器人状态和输出动作向量为零
  robotState = CustomTypes::zeroState(_jointNum);
  robotAction = CustomTypes::zeroAction(_jointNum);
  
  if (!cfg_->cmd_init.isZero()) {
      robotState.targetVelocity = cfg_->cmd_init;
      FRC_INFO("Initial target velocity: " << robotState.targetVelocity.transpose()); 
  }
}

void StateMachine::run(){
  Timer _loopTimer(_policyDt); // 创建一个定时器，周期是 _policyDt 秒（比如 0.01s，表示 100Hz）
  // while(_loopTimer.getMs() < 100) _loopTimer.wait(); // 等待系统至少运行 100ms 再开始主循环，常用于启动缓冲/初始化等待。
  while(_isRunning){
    step();
    _loopTimer.wait();  // 这个函数让状态机以固定的时间间隔 _policyDt 运行 step() 方法，实现一个定周期的控制循环。
  }
}

void StateMachine::parseRobotStates(){
  //  检查指针不为空，确保你在使用 IMU（陀螺仪）和电机状态前，这些数据结构已经正确初始化和赋值。
  assert(_gyroStates != nullptr);
  assert(_motorStates != nullptr);
  // 把电机状态中的时间戳拷贝到 robotState，用于后续时间同步、调试、记录。
  robotState.timestamp = _motorStates->data.timestamp;
  // 读取电机状态
  // memcpy 是 C/C++ 中的一个标准函数，用于在内存中复制一段数据块。它的全称是 memory copy。
  // void* memcpy(void* dest, const void* src, size_t count);
  // dest：目标地址（复制到哪儿） src：源地址（从哪儿复制） count：要复制的字节数
  memcpy(robotState.motorPosition.data(), _motorStates->data.position, _jointNum * sizeof(float));
  memcpy(robotState.motorVelocity.data(), _motorStates->data.velocity, _jointNum * sizeof(float));
  memcpy(robotState.motorTorque.data(), _motorStates->data.ampere, _jointNum * sizeof(float));

  // 📐 读取 IMU 姿态数据
  memcpy(robotState.baseRpy.data(), _gyroStates->gyro.rpy, 3 * sizeof(float));
  memcpy(robotState.baseRpyRate.data(), _gyroStates->gyro.rpy_rate, 3 * sizeof(float));
  memcpy(robotState.baseAcc.data(), _gyroStates->gyro.acc, 3 * sizeof(float));

  // 🔁 姿态变换：欧拉角 → 矩阵 / 四元数
  robotState.baseRotMat = ori::rpyToRotMat(robotState.baseRpy);
  robotState.baseQuat = ori::rpyToQuat(robotState.baseRpy);
  // 🌍 坐标变换：角速度从机体坐标系 → 世界坐标系
  robotState.baseRpyRate_w = robotState.baseRotMat.transpose() * robotState.baseRpyRate;
}

void StateMachine::stop() { _isRunning = false; }

// 你的函数 StateMachine::updateCommands() 是用于根据用户输入（键盘或手柄）更新机器人的目标速度和转向目标（yaw）的逻辑模块。
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
      Vec3f targLinVel_b = robotState.targetVelocity + deltaVelTarg;
      // 也就是说，你是希望在原有目标速度基础上累加新的速度输入，然后再限制最大速度。
      // robotState.targetVelocity = targLinVel_b.cwiseMin(maxVelCmd).cwiseMax(-maxVelCmd);
      robotState.targetVelocity = targLinVel_b
                                  .cwiseMin(maxVelCmd)
                                  .cwiseMax(-maxVelCmd);
      // 逐个分量限制目标速度在 [-maxVelCmd, +maxVelCmd] 范围内；
      // 防止用户按太多次键盘速度无限加上去。

      // 如果目标速度已经非常小，就设为 0； 起到“去抖动”的效果（防止机器人一直小幅抖动/晃动）。
      if (robotState.targetVelocity.norm() < 1e-2) robotState.targetVelocity.setZero();
       // ✅ 输出当前目标速度
      FRC_INFO("[KEYBOARD] Updated target velocity: "
              << robotState.targetVelocity.transpose());
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

    // FRC_INFO("[JYIF.rbtCtrl] Pressed key: '" << *_keyState << "'"
    //      << " | Target Velocity [m/s] = [" << robotState.targetVelocity.transpose() << "]" // 这是机器人期望的线速度向量，单位是 m/s； vz: 通常为 0（地面机器人没垂直运动）；
    //      << " | Yaw Target [rad] = " << yawTarg // 表示你当前设定的期望朝向角度（目标 yaw），单位是弧度； ✅ 用于让机器人朝向某个方向行走或保持方向。
    //      << " | Yaw Rate Cmd [rad/s] = " << robotState.targetOmega[2]); // 表示当前输出的目标角速度，用于驱动机器人旋转； ✅ 控制器会尽力让机器人朝向 yawTarg，以这个角速度旋转。
         // 名称	代表的是什么	属于哪一层逻辑	单位
         // Yaw Target [rad]	想让机器人朝向的绝对角度	🧠 高层策略/决策	弧度（rad）
         // Yaw Rate Cmd [rad/s]	当前控制器输出的转向速度	⚙️ 控制器/执行器	角速度（rad/s）
  }
  
  // ✅ 控制 yaw 角速度，基于当前 yaw 与目标 yaw 的差值，限制在 [-0.4, 0.4] 区间。
  // ✅ 根据当前 yaw（朝向）与目标 yaw（yawTarg）之间的角度差，生成一个“合适的转向角速度”命令 targetOmega[2]，用于让机器人朝目标方向慢慢转过去。
  // yawTarg 是你想让机器人朝向的角度（由按键/手柄决定）； robotState.baseRpy[2] 是机器人当前的实际 yaw；两者做差，得到需要转的角度 deltaYaw（单位是弧度）；
  deltaYaw = yawTarg - robotState.baseRpy[2];
  // 2️⃣ 生成目标角速度（控制量）1.5f * deltaYaw 是一个简单的比例控制器（P控制），表示："当前误差越大，角速度就越大" 但为了安全和稳定，做了限幅：所以角速度被限制在 [-0.4, 0.4] rad/s 范围内。
  robotState.targetOmega[2] = clip(1.5f * deltaYaw, 0.4f,-0.4f);
  // 3️⃣ 抖动过滤（很小就设为 0）如果目标角速度非常小（低于 0.01 rad/s），就将其清零； 避免机器人在已经对准目标方向时还在“微抖动”地转动。
  if (robotState.targetOmega.norm() < 1e-2) robotState.targetOmega.setZero();
}

void StateMachine::packRobotAction(){
  assert(_motorTargets != nullptr);
  _motorTargets->data.timestamp = robotAction.timestamp;
  memcpy(_motorTargets->data.position, robotAction.motorPosition.data(), _jointNum * sizeof(float));
  memcpy(_motorTargets->data.velocity, robotAction.motorVelocity.data(), _jointNum * sizeof(float));
  memcpy(_motorTargets->data.ampere, robotAction.motorTorque.data(), _jointNum * sizeof(float));
  memcpy(_motorTargets->data.kp, robotAction.kP.data(), _jointNum * sizeof(float));
  memcpy(_motorTargets->data.kd, robotAction.kD.data(), _jointNum * sizeof(float));
}