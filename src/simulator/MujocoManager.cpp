// MujocoManager.cpp
#include "simulator/MujocoManager.h"
#include "utility/logger.h"
#include "utility/timer.h"
#include "utility/orientation_tools.h"
#include <cstring>
#include <cmath>
#include <thread>

MujocoManager::MujocoManager(std::shared_ptr<const BaseRobotConfig> cfg,
                             jointCMD* jointCMDPtr,
                             robotStatus* robotStatusPtr)
    : cfg_(cfg),
      robotStatusPtr_(robotStatusPtr),
      jointCMDPtr_(jointCMDPtr),
      robotName_(cfg->robot_name),
      control_dt_(cfg->getPolicyDt()),
      simulation_dt_(cfg->simulation_dt)
      // effort_limit
{
  FRC_INFO("[MjcMgr.Const] Creating MuJoCo simulation for robot: " << robotName_);
  initWorld();
  // moveToDefaultPose();
  initState();
  updateRobotState();
  launchServer();
  FRC_INFO("[MjcMgr.Const] Ready.");
}

void MujocoManager::initWorld() {
  char error[1000] = "";
  mj_model_ = mj_loadXML(cfg_->xml_path.c_str(), nullptr, error, 1000);
  if (!mj_model_) {
    FRC_ERROR("[MjcMgr.initWorld] Failed to load XML: " << error);
    std::exit(1);
  }
  mj_data_ = mj_makeData(mj_model_);
  mj_model_->opt.timestep = simulation_dt_;

  gcDim_ = mj_model_->nq; // nq 是 广义坐标（generalized coordinates） 的维度。
  gvDim_ = mj_model_->nv; // nv 是 广义速度（generalized velocities） 的维度。
  jointDim_ = mj_model_->nu; // nu 是 控制输入（actuator）的数量，即 ctrl 数组的长度。

  FRC_INFO("[MjcMgr.initWorld] Loaded robot: " << robotName_ << " with XML: " << cfg_->xml_path.c_str());
  FRC_INFO("[MjcMgr.initWorld] generalize Coordinate dimensions: " << gcDim_);
  FRC_INFO("[MjcMgr.initWorld] generalize Vel dimensions: " << gvDim_);
  FRC_INFO("[MjcMgr.initWorld] Joint Num: " << jointDim_);
  // ✅ 输出导入后的 qpos
  FRC_INFO("[MjcMgr.initWorld] Initial qpos from XML:");
  std::ostringstream oss;
  for (int i = 0; i < gcDim_; ++i)
    oss << mj_data_->qpos[i] << " ";
  FRC_INFO(oss.str());
}

void MujocoManager::moveToDefaultPose() {
  FRC_INFO("[MjcMgr.moveToDefaultPose] Setting default initial pose...");

  // 默认 root 位置和姿态（单位四元数）
  Eigen::Vector3d root_xyz(0.0, 0.0, 0.80);  // 站立高度
  Eigen::Vector4d root_quat(1.0, 0.0, 0.0, 0.0);  // 无旋转：正立朝上

  // 默认关节角度
  const auto& default_angles = cfg_->default_angles;
  assert(default_angles.size() == jointDim_);

  {
    std::lock_guard<std::mutex> lock(state_lock_);

    // 🟡 打印修改前的 qpos 值
    FRC_INFO("[MjcMgr.moveToDefaultPose] --- BEFORE SETTING ---");
    FRC_INFO("qpos:");
    std::ostringstream qpos_before;
    for (int i = 0; i < gcDim_; ++i)
      qpos_before << mj_data_->qpos[i] << " ";
    FRC_INFO(qpos_before.str());

    // 设置 qpos: root position (0–2), root quaternion (3–6), joint position (7+)
    for (int i = 0; i < 3; ++i) mj_data_->qpos[i] = root_xyz[i];
    for (int i = 0; i < 4; ++i) mj_data_->qpos[3 + i] = root_quat[i];
    for (int i = 0; i < jointDim_; ++i) mj_data_->qpos[7 + i] = default_angles[i];

    // 清零速度
    for (int i = 0; i < gvDim_; ++i) mj_data_->qvel[i] = 0.0;

    // 清零控制输入
    for (int i = 0; i < jointDim_; ++i) mj_data_->ctrl[i] = 0.0;

    // 应用新的状态（重要）
    mj_forward(mj_model_, mj_data_);

    // ✅ 可选：多步 mj_step 来稳定系统（比如站立机器人）
    // for (int i = 0; i < 30; ++i) {
    //   mj_step(mj_model_, mj_data_);
    //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    // 🟢 打印设置后的 qpos
    FRC_INFO("[MjcMgr.moveToDefaultPose] --- AFTER SETTING ---");
    std::ostringstream qpos_after;
    for (int i = 0; i < gcDim_; ++i)
      qpos_after << mj_data_->qpos[i] << " ";
    FRC_INFO(qpos_after.str());

    FRC_INFO("Root pos: " << root_xyz.transpose());
    FRC_INFO("Root quat: " << root_quat.transpose());
    FRC_INFO("Joint pos: " << default_angles.transpose());
    FRC_INFO("[MjcMgr.moveToDefaultPose] Pose applied.");
  }
}

void MujocoManager::initState() {
  // ① 机器人状态变量
  gc_.setZero(gcDim_);      //  当前 generalized coordinate（广义坐标，位置）
  gv_.setZero(gvDim_);      //  当前 generalized velocity（广义速度）

  // ② 控制增益
  jointPGain = cfg_->kP;
  jointDGain = cfg_->kD;

  // ③ 动作目标
  pTarget = cfg_->default_angles;// desired position（用于位置控制）
  FRC_INFO("[MjcMgr.initState] default_angles: " << cfg_->default_angles.transpose());
  vTarget.setZero(jointDim_); // desired velocity（用于速度控制）

  // ④ 力矩命令
  tauCmd.setZero(jointDim_); //	最终输出的控制力矩（全体）
}

void MujocoManager::updateRobotState() {
  Eigen::VectorXf positionVec, velocityVec;
  float timestamp;

  {
    std::lock_guard<std::mutex> stateLock(state_lock_);  // 自动上锁，作用域结束自动释放
    gc_ = Eigen::Map<Eigen::VectorXd>(mj_data_->qpos, gcDim_);
    gv_ = Eigen::Map<Eigen::VectorXd>(mj_data_->qvel, gvDim_);
    positionVec = gc_.cast<float>();
    velocityVec = gv_.cast<float>();
    timestamp = mj_data_->time;
  }

  if (robotStatusPtr_) {
    std::memcpy(robotStatusPtr_->data.position, positionVec.data(), gcDim_ * sizeof(float));
    std::memcpy(robotStatusPtr_->data.velocity, velocityVec.data(), gvDim_ * sizeof(float));
    robotStatusPtr_->data.timestamp = timestamp;
  }
}

void MujocoManager::launchServer() {
  if (!glfwInit()) mju_error("Could not initialize GLFW");
  window_ = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
  if (!window_) mju_error("Could not create GLFW window");
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultScene(&scn_);
  mjr_defaultContext(&con_);
  mjv_makeScene(mj_model_, &scn_, 1000);
  mjr_makeContext(mj_model_, &con_, mjFONTSCALE_150);

  FRC_INFO("[MjcMgr.launchServer] GLFW Initialized");
  FRC_INFO("[MjcMgr.launchServer] GLFW Window: " << window_);
}

void MujocoManager::run() {
  Timer controlTimer(control_dt_);
  while (!glfwWindowShouldClose(window_) && running_) {
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

Eigen::VectorXf pd_control(const Eigen::VectorXf& target_q, const Eigen::VectorXf& q,
                           const Eigen::VectorXf& kp, const Eigen::VectorXf& target_dq, 
                           const Eigen::VectorXf& dq, const Eigen::VectorXf& kd) {
    return kp. cwiseProduct(target_q - q) + kd.cwiseProduct(target_dq - dq);
}

void MujocoManager::integrate() {
  Timer worldTimer(control_dt_);
  while (!glfwWindowShouldClose(window_) && running_) {
    for (int i = 0; i < int(control_dt_ / simulation_dt_); i++) {
      {   // or mjcb_control 简化结构
        std::lock_guard<std::mutex> stateLock(state_lock_);  // 自动锁定 state
        mj_step1(mj_model_, mj_data_); // step1: 更新状态，获得最新 qpos/qvel/contact 等
      }
      {
        // Apply PD control
        std::lock_guard<std::mutex> actionLock(action_lock_);  // 自动锁定 action
        Eigen::VectorXf q = Eigen::Map<Eigen::VectorXd>(mj_data_->qpos + 7, jointDim_).cast<float>();
        Eigen::VectorXf dq = Eigen::Map<Eigen::VectorXd>(mj_data_->qvel + 6, jointDim_).cast<float>();
        tauCmd = pd_control(pTarget, q, jointPGain, vTarget, dq, jointDGain);
        // for (int i = 0; i < tauCmd.size(); ++i) {
        //   tauCmd[i] = std::min(std::max(tauCmd[i], -100.0f), 100.0f);
        // }
        for (int i = 0; i < jointDim_; ++i) mj_data_->ctrl[i] = tauCmd[i];
        // FRC_INFO("tauCmd: " << tauCmd.transpose());
      }
      {
        std::lock_guard<std::mutex> stateLock(state_lock_);  // 自动锁定 state
        mj_step2(mj_model_, mj_data_);  // step2: 推进仿真
      }
    }
    if (!isStatesReady) isStatesReady = true;
    worldTimer.wait();
  }
}

void MujocoManager::renderLoop() {
  // 每秒渲染频率
  const float render_dt = 1.0 / 120.0;
  Timer renderTimer(render_dt);

  while (!glfwWindowShouldClose(window_) && running_) {
    {
      std::lock_guard<std::mutex> lock(state_lock_);
      mjv_updateScene(mj_model_, mj_data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
      int width, height;
      glfwGetFramebufferSize(window_, &width, &height);
      mjr_render({0, 0, width, height}, &scn_, &con_);
    }
    glfwSwapBuffers(window_);
    glfwPollEvents();
    renderTimer.wait();
  }
}

MujocoManager::~MujocoManager() {
  if (mj_data_) mj_deleteData(mj_data_);
  if (mj_model_) mj_deleteModel(mj_model_);
  mjv_freeScene(&scn_);
  mjr_freeContext(&con_);
  if (window_) glfwDestroyWindow(window_);
  glfwTerminate();
}

