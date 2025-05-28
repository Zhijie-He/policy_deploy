#include "simulator/RaisimManager.h"
#include "utility/logger.h"
#include "utility/timer.h"
#include "utility/orientation_tools.h"

RaisimManager::RaisimManager(std::shared_ptr<const RobotConfig> cfg, GyroData* gyroPtr, jointStateData* motorStatePtr, jointTargetData* motorTargetPtr) :
  cfg_(cfg), 
  gyro_data_(gyroPtr), // 保存外部传入的传感器 / 控制器数据指针（实现外部共享）
  motorReadingBuf_(motorStatePtr),
  motorCommandBuf_(motorTargetPtr){

  simTime = new double(0);
  
  // 🕹️ 仿真参数设置
  robotName_ = cfg_->robot_name;
  control_dt_ = cfg_->getPolicyDt();
  simulation_dt_ = cfg_->simulation_dt;
  isFixedBase = cfg_->on_rack;
  
  FRC_INFO("[RsmMgr.Const] Create Raisim");
  // 3. 🌍 创建物理世界
  world_ = new raisim::World();
  world_->setTimeStep(simulation_dt_); // 代表你希望仿真器每一步推进的真实时间 // 每次调用 world_->integrate1()，仿真世界推进的物理时间步长（单位：秒）。

  // 可以选择不同地形环境：
  if (cfg_->world_type == "plain") {
    world_->addGround(0);
  } 

  std::string urdfFullPath = std::string(PROJECT_SOURCE_DIR) + "/" + cfg_->urdf_path;
  robot_ = world_->addArticulatedSystem(urdfFullPath); // 向 Raisim 仿真世界中添加一个“关节型机器人”模型（articulated system），并返回一个指向它的对象指针 robot_。
  FRC_INFO("[RsmMgr.Const] Loaded robot: " << robotName_ << " with URDF: " << urdfFullPath);

  // 打印机器人中“可动关节（movable joints）”的名称列表，并且是“按顺序”的。这个顺序对应于你控制指令中 joint position / velocity / torque 向量中的排列顺序。
  // 是 RaiSim 提供的 API，它是 raisim::ArticulatedSystem 类的一个成员函数。
  robot_->printOutMovableJointNamesInOrder();
  FRC_INFO("[RsmMgr.Const] TotalMass: " << robot_->getTotalMass());

  // 这段代码来自 Raisim API，用于获取机器人模型的自由度信息。
  gcDim_ = int(robot_->getGeneralizedCoordinateDim()); // gc	Generalized Coordinates	广义坐标（位置）  获取 generalized coordinate（广义坐标）向量的维度
  gvDim_ = int(robot_->getDOF()); // gv	Generalized Velocities	广义速度 获取 generalized velocity 向量的维度
  // getGeneralizedCoordinateDim 包含： base position (x, y, z) base orientation (quaternion: qw, qx, qy, qz)  joint positions => 总共维度 = 7 (base pose) + joint DOFs
  // 🔹 getDOF() → gvDim_ 获取 generalized velocity 向量的维度 包含： base linear velocity (3) base angular velocity (3) joint velocities 总共维度 = 6 (base vel) + joint DO
  // 对于 非 fixed base（floating base）机器人，gcDim_ 和 gvDim_ 并不相等。
  // gcDim_	7 (base pose) + jointDim_	广义坐标：位置 + 姿态（四元数）+ 关节位置
  // gvDim_	6 (base twist) + jointDim_	广义速度：线速度 + 角速度 + 关节速度
  // gcDim_ ≠ gvDim_
  // ✅ 只有在固定基座时才相等 
  // 如果是 floating base → gcDim_ > gvDim_（差 1）
  // 如果是 fixed base → gcDim_ == gvDim_ 固定基座机器人中，gcDim_ == gvDim_ == jointDim_，因为没有 base 的自由度（DOF）被建模为变量，所以只包含关节的状态。

  FRC_INFO("[RsmMgr.Const] generalize Coordinate dimensions: " << gcDim_);
  FRC_INFO("[RsmMgr.Const] generalize Vel dimensions: " << gvDim_);

  // 是为了在 固定基座 和 浮动基座（floating base） 机器人中正确计算 关节自由度数量（jointDim_）。
  // jointDim_	我们需要的真正关节数量	即机器人真正可控关节的 DOF
  if (isFixedBase)
      jointDim_ = gvDim_; // 固定基座没有 x/y/z + roll/pitch/yaw 这些自由度 所以 “没有 base” 实际上是 “没有 floating base DOF”
  else
      jointDim_ = gcDim_ - 7; // gcDim_ = 7 + jointDim_ （7 = 3 position + 4 quaternion） jointDim_ = gcDim_ - 7 

  // ① 机器人状态变量
  gc_init_.setZero(gcDim_);  // 初始位置姿态，用于 reset 或 startup
  gv_init_.setZero(gvDim_); // 初始速度，一般为全 0
  gc_.setZero(gcDim_); //	当前 generalized coordinate（广义坐标，位置）
  gv_.setZero(gvDim_); // 当前 generalized velocity（广义速度）
  gv_prev_.setZero(gvDim_); // 	上一时刻速度，用于估计加速度 / 阻尼控制等
  // ② 控制增益
  jointPGain.setZero(gvDim_);
  jointDGain.setZero(gvDim_);
  // ③ 动作目标：pTarget / vTarget
  pTarget.setZero(gcDim_); // desired position（用于位置控制）
  vTarget.setZero(gvDim_); // desired velocity（用于速度控制）
  // ④ 力矩命令
  tauCmd.setZero(gvDim_); //	最终输出的控制力矩（全体）
  tauCmd_joint.setZero(jointDim_); // 仅关节部分力矩
  // 5 接触时的总力/力矩信息 将 contactForce 向量初始化为长度为 6 的零向量，用于存储机器人与地面（或其他物体）接触时的总力/力矩信息。
  contactForce.setZero(6);
  // 分量	意义	单位
  // Fx, Fy, Fz	线性力（x/y/z 方向）	牛顿 (N)
  // Mx, My, Mz	力矩（绕 x/y/z）	牛顿·米 (Nm)

  isRopeHanging = true; // isRopeHanging = true 表示这个 rope（绳子）对象被设置为悬挂状态，也就是： 被某个锚点固定住，允许下垂、摆动、模拟重力作用，但不自由漂浮。
  if (robotName_ == "HuOreo") {
    // 初始化状态  gc_init_ 本身就包含了高度。   // ⬅️ ⬅️ ⬅️ z = 1.16，就是 base 的高度
    gc_init_ << 0, 0, 1.16,
     1, 0.0, 0.0, 0.0,
    -0.08, -0.06, -0.45, 0.94, -0.48, 0.06,
    0.08, 0.06, -0.45, 0.94, -0.48, -0.06,
    0, 0, 0, 0, 0, 0, 0, 0, 0;
    //  设置 PD 控制增益
    jointPGain << 0, 0, 0, 0, 0, 0,
        500, 300, 500, 500, 200, 200,
        500, 300, 500, 500, 200, 200,
        800, 180, 180, 180, 180, 180, 180, 180, 180;
    jointDGain << 0, 0, 0, 0, 0, 0,
        5, 3., 10, 10, 4, 4,
        5, 3., 10, 10, 4, 4,
        16, 2, 2, 2, 3, 2, 2, 2, 3;
    // rope 高度参数
    _ropeHeight = 0.94;

    // toe 名称对应的 body/frame 索引
    shankBodyIdxs[0] = robot_->getBodyIdx("l_toe_Link");
    shankBodyIdxs[1] = robot_->getBodyIdx("r_toe_Link");
    shankFrameIdxs[0] = robot_->getFrameIdxByName("l_toe");
    shankFrameIdxs[1] = robot_->getFrameIdxByName("r_toe");

  } else if (robotName_ == "g1"){
     // 初始化状态  gc_init_ 本身就包含了高度。   // ⬅️ ⬅️ ⬅️ z = 1.16，就是 base 的高度
    gc_init_ << 0, 0, 0.79,
     1.0, 0.0, 0.0, 0.0,
    -0.1,  0.0,  0.0,  0.3, -0.2, 0.0, 
    -0.1,  0.0,  0.0,  0.3, -0.2, 0.0;
    //  设置 PD 控制增益
    jointPGain << 0, 0, 0, 0, 0, 0,
       100, 100, 100, 150, 40, 40,
       100, 100, 100, 150, 40, 40;
    jointDGain << 0, 0, 0, 0, 0, 0,
        2, 2, 2, 4, 2, 2,
        2, 2, 2, 4, 2, 2;
    // rope 高度参数
    _ropeHeight = 0.94;

    // toe 名称对应的 body/frame 索引
    shankBodyIdxs[0] = robot_->getBodyIdx("left_ankle_roll_link");
    shankBodyIdxs[1] = robot_->getBodyIdx("right_ankle_roll_link");
    shankFrameIdxs[0] = robot_->getFrameIdxByName("left_ankle_roll_joint");
    shankFrameIdxs[1] = robot_->getFrameIdxByName("right_ankle_roll_joint");
  }

  robot_->setName(robotName_); // 给机器人对象命名（Raisim 内部管理用途）
  robot_->setState(gc_init_, gv_init_); // 设置机器人初始状态
  robot_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE); // FORCE_AND_TORQUE → 你将完全控制关节的力矩输入（如 τ = PD + policy）
  robot_->setPGains(Eigen::VectorXd::Zero(gvDim_)); // 关闭 Raisim 自带的内部 PD 控制器（默认是开的）
  robot_->setDGains(Eigen::VectorXd::Zero(gvDim_));
  robot_->setGeneralizedForce(tauCmd.cast<double>()); // 将你准备好的 τ（力矩）发送到 Raisim 仿真引擎
  robot_->updateKinematics(); // 强制更新机器人当前的动力学信息
  robot_->getState(gc_, gv_); // 把当前机器人状态（位姿+速度）读出来，赋值到你本地的 gc_, gv_ 中
  gf_ = robot_->getGeneralizedForce().e();  // 获取当前机器人关节上的 总力矩输出 分析是否输出超限  	当前 generalized force（力矩）
  
  updateRobotState(); //  状态同步函数

  server_ = new raisim::RaisimServer(world_);
  server_->launchServer();
  server_->focusOn(robot_);
  FRC_INFO("[RsmMgr.Const] During running, press '-' or 'Home' on joystick for reset.");
  FRC_INFO("[RsmMgr.Const] Press ENTER to start simulation.");
}

void RaisimManager::updateRobotState() {
  state_lock_.lock();
  // 🔹 1. 关节向量提取（用于下发和日志）
  Eigen::VectorXf jointPosVec = gc_.tail(jointDim_).cast<float>(); // “Joint Position Vector”，即关节位置向量
  Eigen::VectorXf jointVelVec = gv_.tail(jointDim_).cast<float>();
  Eigen::VectorXf jointTauVec = gf_.tail(jointDim_).cast<float>();

  // gc_, gv_, gf_ 是完整的 base + joint 向量
  // .tail(jointDim_) 取的是关节部分（后面 n 个 DOF）
  // cast<float>() 是从 Raisim 默认的 double → 你项目用的 float

  // Transport generalized states into gyroState(STGyro): raw data from IMU include RPY, acceleration and RPY angle velocity
  Vec4d quatBody;
  Vec3d rpy;
  rpy.setZero();
  
  // 🔹 2. IMU 数据更新（非固定基座时）✅ 为什么这么做？ 你要模拟一个“真实 IMU”，而不是直接用 Raisim 内部的 getCOM()
  if (isFixedBase) {
    for (auto & i : gyro_data_->gyro.buffer)  memset(i, 0, 3 * sizeof(float));   // 清零 IMU
  } else {
    quatBody = gc_.segment(3, 4); // 🔹1. 提取 base 的四元数姿态
    rpy = ori::quatToRPY(quatBody); // 🔹2. 姿态角 RPY 计算 把四元数转为 roll-pitch-yaw 可删除
    Mat3d rotMat = ori::quaternionToRotationMatrix(quatBody); // 🔹3. 获取旋转矩阵（base → world）
    rotMat = robot_->getBaseOrientation().e().transpose(); // 更准确，用 Raisim 自带方法
    rpy = ori::rotationMatrixToRPY(rotMat); // 🔹4. 最终 RPY 姿态角再计算一次（用 rotMat）
    Vec3d omega_w = gv_.segment(3, 3); // 🔹5. 姿态角速度计算（RPY rate）  gv_[3:6] 是 base 的角速度（在 world 坐标系下）
    Vec3d rpy_rate = rotMat * omega_w; // 转为 body frame 中的 RPY rate 用 rotMat 转换为 body 坐标下的角速度（近似为 RPY rate）
    // Vec3d rpy_rate = ori::globalAngularVelocityToDrpyXyz(rpy, omega_w);
    Vec3d acc_b = rotMat * ((gv_.head(3) - gv_prev_.head(3)) / control_dt_ - world_->getGravity().e()); // 🔹6. 加速度计算（body frame 下）
    // 得到 acc_b 是：在 base 坐标系下的线加速度
    
    // 🔹7. 写入 IMU 数据结构（共享内存结构）
    for (int i = 0; i < 3; i++) {
      gyro_data_->gyro.rpy[i] = float(rpy[i]); // base 姿态角（roll, pitch, yaw）
      gyro_data_->gyro.rpy_rate[i] = float(rpy_rate[i]); // 姿态角变化速度（近似）
      gyro_data_->gyro.acc[i] = float(acc_b[i]); //	base 加速度（IMU 的线加速度读数）
    }
  }
  state_lock_.unlock();

  // 把 Raisim 仿真器中的机器人关节状态（位置、速度、力矩）写入共享的控制数据结构 motorReadingBuf_，用于之后的： 控制器接收当前状态（如 PD 控制或神经网络推理）  DDS/ROS2 通信发布
  memcpy(motorReadingBuf_->data.position, jointPosVec.data(), jointDim_ * sizeof(float)); // 🔹1. 写入关节位置
  memcpy(motorReadingBuf_->data.velocity, jointVelVec.data(), jointDim_ * sizeof(float)); // 🔹2. 写入关节速度
  memcpy(motorReadingBuf_->data.ampere, jointTauVec.data(), jointDim_ * sizeof(float));  // 🔹3. 写入关节“力矩”
  motorReadingBuf_->data.timestamp = timeSinceEpochUs();  // 🔹4. 写入时间戳 记录当前状态对应的时间（单位是微秒）
  // FRC_INFO("jointPosVec = \n" << jointPosVec.transpose());
  // FRC_INFO("jointVelVec = \n" << jointVelVec.transpose());
  // FRC_INFO("jointTauVec = \n" << jointTauVec.transpose());
  // FRC_INFO("gc_ = \n" << gc_.transpose());
}

void RaisimManager::run() {
  Timer controlTimer(control_dt_);
  while (running_) {
    *simTime = world_->getWorldTime(); // 从 Raisim 世界中获取当前的仿真时间（单位：秒），并写入外部的 simTime 变量中。
    auto actionLatest = std::make_shared<jointTargetData>();
    memcpy(actionLatest.get(), motorCommandBuf_, sizeof(jointTargetData)); // 将一段内存从 motorCommandBuf_ 复制到 actionLatest 指向的内存中 sizeof(jointTargetData)	要复制的字节数，正好是一个结构体的大小
    
    action_lock_.lock(); // 加锁，确保以下多线程数据操作是安全的
    memcpy(pTarget.data()+7, actionLatest->data.position, sizeof(float) * jointDim_);    // 加了偏移 +6 / +7 是为了 跳过前6或7个自由度（如 base DOF），只对关节生效
    memcpy(vTarget.data()+6, actionLatest->data.velocity, sizeof(float) * jointDim_);
    memcpy(tauCmd_joint.data()+6, actionLatest->data.ampere, sizeof(float) * jointDim_);
    memcpy(jointPGain.data()+6, actionLatest->data.kp, sizeof(float) * jointDim_);
    memcpy(jointDGain.data()+6, actionLatest->data.kd, sizeof(float) * jointDim_);
    action_lock_.unlock();
    float timeDiff = (timeSinceEpochUs() - actionLatest->data.timestamp) / 1e3;
    tauCmd.tail(jointDim_) = tauCmd_joint;

    if (isStatesReady) updateRobotState();
    controlTimer.wait();
  }
}

// 最核心的物理主循环
void RaisimManager::integrate() {
  Timer worldTimer(control_dt_);
  while (running_) {
    action_lock_.lock();
    // 🔹控制器部分：PD控制器更新  gc_ 是实际位姿，pTarget 是目标位姿  gv_ 是实际速度，vTarget 是目标速度
    // (pTarget - gc_) 和 (vTarget - gv_) 是 position/velocity 误差 计算出的 tauCmd 是你要下发的力矩（PD 输出）
    tauCmd.tail(jointDim_) = (
                            jointPGain.cwiseProduct((pTarget - gc_.cast<float>()).tail(gvDim_))
                          + jointDGain.cwiseProduct(vTarget - gv_.cast<float>()) 
                        ).tail(jointDim_);
    action_lock_.unlock();

    // Raisim 要求整条力矩向量的长度必须 与 generalized velocity 一致（gvDim_）
    robot_->setGeneralizedForce(tauCmd.cast<double>()); // ✅ 向 Raisim 仿真器下发控制命令（力矩） 相当于告诉仿真器：当前控制周期，我们要对每个自由度施加多大的力矩
    if (skillField) _terrain->environmentalCallback(robot_);

    // ### 🌍 Raisim 仿真推进 作用：推进 Raisim 仿真若干小步（以 simulation_dt_ 为粒度）
    for (int i = 0; i < int(control_dt_ / simulation_dt_); i++) {
      if (server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if (server_) server_->unlockVisualizationServerMutex();
    }
    gv_prev_ = gv_;  // 保存上一次速度 gv_prev_ 以便估算加速度

    state_lock_.lock();
    robot_->getState(gc_, gv_); // 更新 gc_（位置）和 gv_（速度）
    gf_ = robot_->getGeneralizedForce().e(); // 获取当前的关节/基座力矩 gf_（从 Raisim 中）
    state_lock_.unlock();

    // 🔁 人工触发复位（reset） 按键 - 或手柄 Start 会触发一次复位（即重新设置 robot 初始状态）
    if ((keyPtr_ != nullptr && *keyPtr_ == '-') || (joyPtr_ != nullptr && joyPtr_->buttonStart)) {
      FRC_INFO("[Simulator.int] Resetting the robot back to hanging state..");
      reset();
    }

    // 🦶 获取接触力
    getContactForce();

    // 🧭 rope assist 自动落地逻辑
    if (!isStatesReady) isStatesReady = true;
    if (!isFixedBase && isRopeHanging) {
        // 这段是最关键的“机器人从空中挂起 → 检测落地 → 释放绳子”过程。
        raisim::Mat<3, 3> nominalOri{};
        nominalOri.setIdentity();
        raisim::Vec<3> slowVel{0, 0, -0.05};
        double totalGravity = -world_->getGravity()[2] * robot_->getTotalMass();
        // 🪂 模拟慢速下降（让机器人缓缓落地） 判断条件：机器人当前高度高于绳子锚点、并且是朝下运动中
        if (gc_[2] > _ropeHeight && gv_[2] < slowVel[2]) {
          robot_->setBaseVelocity(slowVel);  // 慢慢下落
          robot_->setBaseOrientation(nominalOri); // 姿态保持竖直
        }
        //   The rope assistance stops when the robot's feet first touches the ground.
        // 🦶 触地检测（接触力大于 40% 重力）
        // 条件：0.2 秒之后接触力足够大，说明脚碰地了
        // 一旦落地，标志 isRopeHanging 设为 false → 从此不再干预 base 的运动
        if (world_->getWorldTime() > 0.2 && (contactForce.head(3).norm() > 0.4 * totalGravity || contactForce.tail(3).norm() > 0.4 * totalGravity)) {
          isRopeHanging = false;

          FRC_INFO("Robot gravity" << totalGravity);
          FRC_INFO("Stops assisting landing" << contactForce.head(3).norm() << ", " << contactForce.tail(3).norm());
        }
    }
    // 控制频率节拍器，确保仿真控制周期精确为 control_dt_
    worldTimer.wait();
  }
}

void RaisimManager::reset() {
  tauCmd.setZero();  // 清零控制力矩
  robot_->setState(gc_init_, gv_init_);  // 重置机器人位姿（位置 + 速度）
  robot_->setGeneralizedForce(tauCmd.cast<double>());  // 重新应用 0 力矩

  if (isStatesReady) isStatesReady = false;  // 下一个周期重新更新状态
  if (!isFixedBase) isRopeHanging = true;    // 重启 rope 模式
}

void RaisimManager::getContactForce() {
  double sum = 0.;
  contactForce.setZero(6);

  for (auto& contact : robot_->getContacts()) {
    if (contact.isSelfCollision() || contact.skip())
      continue; /// if the contact is internal, one contact point is set to 'skip'
    for (int i = 0; i < 2; i++) {
      if (shankBodyIdxs[i] == contact.getlocalBodyIndex()) {
        raisim::Vec<3> impulse;
        auto pimpulse = contact.getImpulsePtr();
        if (pimpulse == nullptr) continue;
        impulse = *pimpulse;
        if (isnan(impulse.e().norm())) { RSINFO("Foot Frc " << i << " is Nan.") }
        else if (isinf(impulse.e().norm())) { RSINFO("Foot Frc " << i << " is Inf.") }
        else contactForce.segment(3 * i, 3) += -1 * contact.getContactFrame().e().transpose() * impulse.e() / simulation_dt_;
      }
    }
  }
}
// contactForce: Eigen::VectorXd(6) → [左脚 fx, fy, fz, 右脚 fx, fy, fz