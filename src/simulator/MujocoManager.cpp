// MujocoManager.cpp
#include "simulator/MujocoManager.h"
#include "utility/logger.h"
#include "utility/timer.h"
#include "utility/orientation_tools.h"
#include <cstring>
#include <cmath>

MujocoManager::MujocoManager(std::shared_ptr<const RobotConfig> cfg,
                             GyroData* gyroPtr,
                             jointStateData* motorInputPtr,
                             jointTargetData* motorOutputPtr)
    : cfg_(cfg),
      gyro_data_(gyroPtr),
      motorReadingBuf_(motorInputPtr),
      motorCommandBuf_(motorOutputPtr),
      robotName_(cfg->robot_name),
      control_dt_(cfg->getPolicyDt()),
      simulation_dt_(cfg->simulation_dt),
      isFixedBase(cfg->on_rack) {

  FRC_INFO("[MjcMgr.Const] Creating MuJoCo simulation for robot: " << robotName_);
  initWorld();
  loadRobotModel();
  initState();
  setupRobotProperties();
  updateRobotState();
  launchServer();
  FRC_INFO("[MjcMgr.Const] Ready. Press ENTER to start simulation.");
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
}

void MujocoManager::loadRobotModel() {


}

void MujocoManager::initState() {
  // ① 机器人状态变量
  gc_init_.setZero(gcDim_);  // 初始位置姿态，用于 reset 或 startup
  gv_init_.setZero(gvDim_); //  初始速度，一般为全 0
  gc_.setZero(gcDim_);      //  当前 generalized coordinate（广义坐标，位置）
  gv_.setZero(gvDim_);      //  当前 generalized velocity（广义速度）
  gv_prev_.setZero(gvDim_); // 	上一时刻速度，用于估计加速度 / 阻尼控制等

  // ② 控制增益
  jointPGain.setZero(gvDim_);
  jointDGain.setZero(gvDim_);

  // ③ 动作目标
  pTarget.setZero(gcDim_); // desired position（用于位置控制）
  vTarget.setZero(gvDim_); // desired velocity（用于速度控制）

  // ④ 力矩命令
  tauCmd.setZero(gvDim_); //	最终输出的控制力矩（全体）
  tauCmd_joint.setZero(jointDim_); // 仅关节部分力矩

  // ⑤ 接触力
  contactForce.setZero(6); // Fx, Fy, Fz, Mx, My, Mz
}

void MujocoManager::setupRobotProperties() {
  // 重置仿真状态（清空 mjData）
  mj_resetData(mj_model_, mj_data_);
  for (int i = 0; i < gcDim_; ++i) mj_data_->qpos[i] = gc_init_[i];
  for (int i = 0; i < gvDim_; ++i) mj_data_->qvel[i] = gv_init_[i];
  // 根据 qpos 和 qvel 推进一次前向动力学（生成接触信息等）
  mj_forward(mj_model_, mj_data_);
}

void MujocoManager::updateRobotState() {
  gc_ = Eigen::Map<Eigen::VectorXd>(mj_data_->qpos, gcDim_);
  gv_ = Eigen::Map<Eigen::VectorXd>(mj_data_->qvel, gvDim_);
  gv_prev_ = gv_;

  // Fake IMU from base angular vel and gravity
  if (gyro_data_) {
    for (int i = 0; i < 3; ++i) {
      gyro_data_->gyro.rpy[i] = 0.0f; // Not computed here
      gyro_data_->gyro.rpy_rate[i] = gv_[i + 3];
      gyro_data_->gyro.acc[i] = 0.0f; // Optional to add from qacc
    }
  }

  // Update joint state
  if (motorReadingBuf_) {
    std::memcpy(motorReadingBuf_->data.position, mj_data_->qpos + 7, jointDim_ * sizeof(float));
    std::memcpy(motorReadingBuf_->data.velocity, mj_data_->qvel + 6, jointDim_ * sizeof(float));
    std::memcpy(motorReadingBuf_->data.ampere, mj_data_->ctrl, jointDim_ * sizeof(float));
    motorReadingBuf_->data.timestamp = timeSinceEpochUs();
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
    updateRobotState();

    // Apply PD control
    {
      std::lock_guard<std::mutex> lock(action_lock_);
      for (int i = 0; i < jointDim_; ++i) {
        float q_err = pTarget[i] - mj_data_->qpos[i + 7];
        float dq_err = vTarget[i] - mj_data_->qvel[i + 6];
        tauCmd[i] = jointPGain[i] * q_err + jointDGain[i] * dq_err;
        mj_data_->ctrl[i] = tauCmd[i];
      }
    }

    mj_step(mj_model_, mj_data_);
    mjv_updateScene(mj_model_, mj_data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    mjr_render({0, 0, 1200, 900}, &scn_, &con_);
    glfwSwapBuffers(window_);
    glfwPollEvents();
    controlTimer.wait();
  }
}

void MujocoManager::integrate() {
  // Empty, as run() includes step()
}

void MujocoManager::reset() {
  std::memcpy(mj_data_->qpos, gc_init_.data(), sizeof(double) * gcDim_);
  std::memcpy(mj_data_->qvel, gv_init_.data(), sizeof(double) * gvDim_);
  mj_forward(mj_model_, mj_data_);
}

void MujocoManager::getContactForce() {
  contactForce.setZero();
  // Optional: use data_->cfrc_ext if contacts are enabled
}

MujocoManager::~MujocoManager() {
  if (mj_data_) mj_deleteData(mj_data_);
  if (mj_model_) mj_deleteModel(mj_model_);
  mjv_freeScene(&scn_);
  mjr_freeContext(&con_);
  if (window_) glfwDestroyWindow(window_);
  glfwTerminate();
}