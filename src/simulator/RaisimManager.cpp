#include "simulator/RaisimManager.h"
#include "utility/logger.h"
#include "utility/timer.h"
#include "utility/orientation_tools.h"

RaisimManager::RaisimManager(
                std::shared_ptr<const BaseRobotConfig> cfg,
                jointCMD* jointCMDPtr,
                robotStatus* robotStatusPtr)
    : cfg_(cfg),
      robotStatusPtr_(robotStatusPtr),
      jointCMDPtr_(jointCMDPtr),
      robotName_(cfg->robot_name),
      control_dt_(cfg->getPolicyDt()),
      simulation_dt_(cfg->simulation_dt)
{
  FRC_INFO("[RsmMgr.Const] Create Raisim");
  initWorld();              // 地形 & 仿真环境
  initState();              // 初始化所有状态变量
  setupRobotProperties();   // PD控制、初始状态、命名、控制模式
  updateRobotState();       // 与 motorReadingBuf_ 同步
  launchServer();           // 可视化
  
  FRC_INFO("[RsmMgr.Const] During running, press '-' or 'Home' on joystick for reset.");
  FRC_INFO("[RsmMgr.Const] Press ENTER to start simulation.");
}

void RaisimManager::initWorld() {
  world_ = new raisim::World();
  world_->setTimeStep(simulation_dt_);
  if (cfg_->world_type == "plain") {
    world_->addGround();
  } else if (cfg_->world_type == "multi-skill challenge") {
    skillField = true;
    _terrain = std::make_shared<MultiSkillField>(world_, cfg_->terrain_config_file);
  }
  loadRobotModel();
}

void RaisimManager::loadRobotModel() {
  robot_ = world_->addArticulatedSystem(cfg_->urdf_path);
  robot_->printOutMovableJointNamesInOrder();
  gcDim_ = int(robot_->getGeneralizedCoordinateDim());
  gvDim_ = int(robot_->getDOF());
  jointDim_ = isFixedBase ? gvDim_ : gvDim_ - 6;
  FRC_INFO("[RsmMgr.Const] Loaded robot: " << robotName_ << " with URDF: " << cfg_->urdf_path);
  FRC_INFO("[RsmMgr.Const] TotalMass: " << robot_->getTotalMass());
  FRC_INFO("[RsmMgr.Const] generalize Coordinate dimensions: " << gcDim_);
  FRC_INFO("[RsmMgr.Const] generalize Vel dimensions: " << gvDim_);
  FRC_INFO("[RsmMgr.Const] Joint Num: " << jointDim_);

}

void RaisimManager::initState() {
  // ① 机器人状态变量
  gc_init_.setZero(gcDim_);      //  当前 generalized coordinate（广义坐标，位置）
  gv_init_.setZero(gvDim_);      //  当前 generalized velocity（广义速度）
  gc_.setZero(gcDim_);      //  当前 generalized coordinate（广义坐标，位置）
  gv_.setZero(gvDim_);      //  当前 generalized velocity（广义速度）

  // ② 控制增益
  jointPGain = cfg_->kP;
  jointDGain = cfg_->kD;

  // ③ 动作目标
  pTarget = cfg_->default_angles;// desired position（用于位置控制）
  FRC_INFO("[RaisimManager.initState] default_angles: " << cfg_->default_angles.transpose());
  vTarget.setZero(jointDim_); // desired velocity（用于速度控制）

  // ④ 力矩命令
  tauCmd.setZero(jointDim_); 
  fullTauCmd.setZero(gvDim_); 
}

void RaisimManager::setupRobotProperties() {
  if (robotName_ == "g1"){
    gc_init_ << 0, 0, 0.78, 1.0, 0.0, 0.0, 0.0,
     0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 
     0.0,  0.0,  0.0,  0.0, 0.0, 0.0;
  }

  robot_->setName(robotName_);
  robot_->setState(gc_init_, gv_init_);
  robot_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
  robot_->setPGains(Eigen::VectorXd::Zero(gvDim_));
  robot_->setDGains(Eigen::VectorXd::Zero(gvDim_));
  robot_->setGeneralizedForce(fullTauCmd.cast<double>());
  robot_->updateKinematics();
  robot_->getState(gc_, gv_);
  gf_ = robot_->getGeneralizedForce().e();
}

void RaisimManager::updateRobotState() {
  Eigen::VectorXf positionVec, velocityVec;
  float timestamp;
  {
    std::lock_guard<std::mutex> stateLock(state_lock_);  // 自动上锁，作用域结束自动释放
    positionVec = gc_.cast<float>(); 
    velocityVec = gv_.cast<float>();
    timestamp = static_cast<float>(world_->getWorldTime());
  }

  if (robotStatusPtr_) {
    std::memcpy(robotStatusPtr_->data.position, positionVec.data(), gcDim_ * sizeof(float));
    std::memcpy(robotStatusPtr_->data.velocity, velocityVec.data(), gvDim_ * sizeof(float));
    robotStatusPtr_->data.timestamp = timestamp;
  }
}

void RaisimManager::launchServer() {
  server_ = new raisim::RaisimServer(world_);
  server_->launchServer();
  server_->focusOn(robot_);
}

void RaisimManager::run() {
  Timer controlTimer(control_dt_);
  while (running_) {
    auto action = std::make_shared<jointCMD>();

    {
      std::lock_guard<std::mutex> actionLock(action_lock_);
      memcpy(action.get(), jointCMDPtr_, sizeof(jointCMD));
      memcpy(pTarget.data(), action->data.position, sizeof(float) * jointDim_);
      memcpy(vTarget.data(), action->data.velocity, sizeof(float) * jointDim_);
      memcpy(jointPGain.data(), action->data.kp, sizeof(float) * jointDim_);
      memcpy(jointDGain.data(), action->data.kd, sizeof(float) * jointDim_);
    }

    if (isStatesReady) updateRobotState();
    controlTimer.wait();
  }
}

void RaisimManager::integrate() {
  Timer worldTimer(control_dt_);
  while (running_) {
    {
      std::lock_guard<std::mutex> actionLock(action_lock_);  // 自动锁定 action
      Eigen::VectorXf q = gc_.tail(jointDim_).cast<float>();   // 当前关节位置
      Eigen::VectorXf dq = gv_.tail(jointDim_).cast<float>();  // 当前关节速度
      tauCmd = jointPGain.cwiseProduct(pTarget - q) + jointDGain.cwiseProduct(vTarget - dq);
    }

    // Raisim 要求整条力矩向量的长度必须 与 generalized velocity 一致（gvDim_）
    fullTauCmd.tail(jointDim_) = tauCmd;
    robot_->setGeneralizedForce(fullTauCmd.cast<double>()); 
    if (skillField) _terrain->environmentalCallback(robot_);

    // Raisim 仿真推进 作用：推进 Raisim 仿真若干小步（以 simulation_dt_ 为粒度）
    for (int i = 0; i < int(control_dt_ / simulation_dt_); i++) 
    {
      if (server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if (server_) server_->unlockVisualizationServerMutex();
    }
    
    {
      std::lock_guard<std::mutex> stateLock(state_lock_);  // 自动锁定 action
      robot_->getState(gc_, gv_); // 更新 gc_（位置）和 gv_（速度）
      gf_ = robot_->getGeneralizedForce().e(); // 获取当前的关节/基座力矩 gf_（从 Raisim 中）
    }
    // 控制频率节拍器，确保仿真控制周期精确为 control_dt_
    worldTimer.wait();
  }
}

