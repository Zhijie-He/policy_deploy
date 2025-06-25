// G1Sim2MujocoEnv.cpp
#include "sim2/simulator/g1_sim2mujoco_env.h"
#include "utility/logger.h"
#include "utility/timer.h"
#include "utility/orientation_tools.h"
#include "utility/tools.h"
#include <cstring>
#include <cmath>
#include <thread>

G1Sim2MujocoEnv::G1Sim2MujocoEnv(std::shared_ptr<const BaseRobotConfig> cfg,
                                 std::shared_ptr<StateMachine> state_machine)
    : BaseEnv(cfg, state_machine->getJointCMDBufferPtr(), state_machine->getRobotStatusBufferPtr()),
      robotName_(cfg->robot_name),
      simulation_dt_(cfg->simulation_dt),
      state_machine_(state_machine)
{
  tools::checkMujucoVersion();
  initWorld();
  initState();
  updateRobotState();
  FRC_INFO("[G1Sim2MujocoEnv.Const] Ready.");
}

G1Sim2MujocoEnv::G1Sim2MujocoEnv(std::shared_ptr<const BaseRobotConfig> cfg,
                                 std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr,
                                 std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr)
    : BaseEnv(cfg, jointCMDBufferPtr, robotStatusBufferPtr),
      robotName_(cfg->robot_name),
      simulation_dt_(cfg->simulation_dt) 
{
  tools::checkMujucoVersion();
  initWorld();
  initState();
  updateRobotState();
  FRC_INFO("[G1Sim2MujocoEnv.Const] Ready.");
}

void G1Sim2MujocoEnv::initWorld() {
  char error[1000] = "";
  mj_model_ = mj_loadXML(cfg_->xml_path.c_str(), nullptr, error, 1000);
  if (!mj_model_) {
    FRC_ERROR("[G1Sim2MujocoEnv.initWorld] Failed to load XML: " << error);
    std::exit(1);
  }
  mj_data_ = mj_makeData(mj_model_);
  mj_model_->opt.timestep = simulation_dt_;

  gcDim_ = mj_model_->nq; // nq 是 广义坐标（generalized coordinates） 的维度。
  gvDim_ = mj_model_->nv; // nv 是 广义速度（generalized velocities） 的维度。
  jointDim_ = mj_model_->nu; // nu 是 控制输入（actuator）的数量，即 ctrl 数组的长度。

  FRC_INFO("[G1Sim2MujocoEnv.initWorld] Loaded robot: " << robotName_ << " with XML: " << cfg_->xml_path.c_str());
  FRC_INFO("[G1Sim2MujocoEnv.initWorld] generalize Coordinate dimensions: " << gcDim_);
  FRC_INFO("[G1Sim2MujocoEnv.initWorld] generalize Vel dimensions: " << gvDim_);
  FRC_INFO("[G1Sim2MujocoEnv.initWorld] Joint Num: " << jointDim_);
 
  std::ostringstream oss;
  for (int i = 0; i < gcDim_; ++i)
    oss << mj_data_->qpos[i] << " ";
  FRC_INFO("[G1Sim2MujocoEnv.initWorld] Initial qpos from XML: " <<oss.str());
}

void G1Sim2MujocoEnv::initState() {
  // ① 机器人状态变量
  gc_.setZero(gcDim_);           //  当前 generalized coordinate（广义坐标，位置）
  gv_.setZero(gvDim_);           //  当前 generalized velocity（广义速度）
  joint_torques_.setZero(jointDim_); //  当前 tau

  // ② 控制增益
  // jointPGain.setZero(jointDim_); 
  // jointDGain.setZero(jointDim_); 
  jointPGain = cfg_->kP;
  jointDGain = cfg_->kD;
  
  // ③ 动作目标
  pTarget.setZero(jointDim_); // desired position（用于位置控制）
  vTarget.setZero(jointDim_); // desired velocity（用于速度控制）

  // ④ 力矩命令
  tauCmd.setZero(jointDim_); //	最终输出的控制力矩（全体）
}

void G1Sim2MujocoEnv::launchServer() {
  if (headless_) {
    FRC_INFO("[G1Sim2MujocoEnv.launchServer] Headless mode enabled. Skipping GUI setup.");
    return;
  }

  if (!glfwInit()) mju_error("Could not initialize GLFW");
  window_ = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
  if (!window_) mju_error("Could not create GLFW window");
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
  
  // initialize visualization data structures
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultScene(&scn_);
  mjr_defaultContext(&con_);
  
  // create scene and context
  mjv_makeScene(mj_model_, &scn_, 2000);
  mjr_makeContext(mj_model_, &con_, mjFONTSCALE_150);

  // 给 GLFW 窗口设置 user pointer
  glfwSetWindowUserPointer(window_, this);

  // 鼠标拖动控制视角
  glfwSetCursorPosCallback(window_, [](GLFWwindow* window, double xpos, double ypos) {
    static bool first_move = true;
    static double lastx = 0.0, lasty = 0.0;

    auto* mgr = static_cast<G1Sim2MujocoEnv*>(glfwGetWindowUserPointer(window));
    if (!mgr) return;

    if (first_move) {
      lastx = xpos;
      lasty = ypos;
      first_move = false;
    }

    // 判断哪个按键被按下，决定动作类型
    mjtMouse action = mjMOUSE_NONE;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
      action = mjMOUSE_ROTATE_V;
    else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
      action = mjMOUSE_MOVE_V;
    else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS)
      action = mjMOUSE_ZOOM;

    if (action != mjMOUSE_NONE) {
      int width, height;
      glfwGetWindowSize(window, &width, &height);
      double dx = xpos - lastx;
      double dy = ypos - lasty;
      mjv_moveCamera(mgr->mj_model_, action, dx / height, dy / height, &mgr->scn_, &mgr->cam_);
    }

    lastx = xpos;
    lasty = ypos;
  });

  // 鼠标滚轮缩放
  glfwSetScrollCallback(window_, [](GLFWwindow* window, double xoffset, double yoffset) {
    auto* mgr = static_cast<G1Sim2MujocoEnv*>(glfwGetWindowUserPointer(window));
    if (!mgr) return;
    mjv_moveCamera(mgr->mj_model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &mgr->scn_, &mgr->cam_);
  });

  FRC_INFO("[G1Sim2MujocoEnv.launchServer] GLFW Initialized");
  FRC_INFO("[G1Sim2MujocoEnv.launchServer] GLFW Window: " << window_);
}

void G1Sim2MujocoEnv::updateRobotState() {
  Eigen::VectorXf positionVec, velocityVec, jointTorquesVec;
  float timestamp;
  {
    std::lock_guard<std::mutex> stateLock(state_lock_);  // 自动上锁，作用域结束自动释放
    gc_ = Eigen::Map<Eigen::VectorXd>(mj_data_->qpos, gcDim_);
    gv_ = Eigen::Map<Eigen::VectorXd>(mj_data_->qvel, gvDim_);
    joint_torques_  = Eigen::Map<Eigen::VectorXd>(mj_data_->ctrl, jointDim_);
    positionVec = gc_.cast<float>();
    velocityVec = gv_.cast<float>();
    jointTorquesVec = joint_torques_.cast<float>();
    timestamp = mj_data_->time;
  }

  if (robotStatusBufferPtr_) {
    robotStatus status;
    memcpy(status.data.position, positionVec.data(), gcDim_ * sizeof(float));
    memcpy(status.data.velocity, velocityVec.data(), gvDim_ * sizeof(float));
    memcpy(status.data.jointTorques, jointTorquesVec.data(), jointDim_ * sizeof(float));
    status.data.timestamp = timestamp;
    robotStatusBufferPtr_->SetData(status);
  }
}

void G1Sim2MujocoEnv::step() {
  // Timer controlTimer(control_dt_);
  RateLimiter controlTimer(1.0 / control_dt_, "mujoco main loop");
  while ((headless_ || !glfwWindowShouldClose(window_)) && running_){

    if(state_machine_) state_machine_->step();

    auto actionPtr = jointCMDBufferPtr_->GetData();
    while (!actionPtr) { // 等待policy 传递action
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      actionPtr = jointCMDBufferPtr_->GetData();  // 重新尝试获取
    }
    
    {
      std::lock_guard<std::mutex> actionLock(action_lock_);  // 自动锁定 action
      const auto& cmd = *actionPtr; // 直接解引用使用数据（推荐写法）
      memcpy(pTarget.data(), cmd.data.position, sizeof(float) * jointDim_);
      memcpy(vTarget.data(), cmd.data.velocity, sizeof(float) * jointDim_);
      memcpy(jointPGain.data(), cmd.data.kp, sizeof(float) * jointDim_);
      memcpy(jointDGain.data(), cmd.data.kd, sizeof(float) * jointDim_);
    }
    
    updateRobotState();
    controlTimer.wait();
  }
}

void G1Sim2MujocoEnv::run() {
  if (headless_) {
      FRC_INFO("[G1Sim2MujocoEnv.run] Headless mode. Running step loop without rendering.");
      std::thread step_thread(&G1Sim2MujocoEnv::step, this);
      step_thread.join(); 
      return;
  }
  launchServer();
  RateLimiter renderTimer(1.0 / control_dt_, "mujoco render loop");
  std::thread step_thread(&G1Sim2MujocoEnv::step, this);
  while (!glfwWindowShouldClose(window_) && running_) {
    integrate();
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

  step_thread.join();
}

void G1Sim2MujocoEnv::integrate() {
  for (int i = 0; i < int(control_dt_ / simulation_dt_); i++) {
    {   // or mjcb_control 简化结构
      std::lock_guard<std::mutex> stateLock(state_lock_);  
      mj_step1(mj_model_, mj_data_); // step1: 更新状态，获得最新 qpos/qvel/contact 等
    }
    { 
      std::lock_guard<std::mutex> actionLock(action_lock_);
      Eigen::VectorXf joint_pos = Eigen::Map<Eigen::VectorXd>(mj_data_->qpos + 7, jointDim_).cast<float>();
      Eigen::VectorXf joint_vel = Eigen::Map<Eigen::VectorXd>(mj_data_->qvel + 6, jointDim_).cast<float>();
      tauCmd = tools::pd_control(pTarget, joint_pos, jointPGain, vTarget, joint_vel, jointDGain);
      for (int i = 0; i < tauCmd.size(); ++i) {
        float limit = cfg_->effort_limit[i];
        if(std::isfinite(limit)){
          tauCmd[i] = std::clamp(tauCmd[i], -limit, limit);
        }
      }
      for (int i = 0; i < jointDim_; ++i) mj_data_->ctrl[i] = tauCmd[i];
      // FRC_INFO("tauCmd: " << tauCmd.transpose());
    }
    {
      std::lock_guard<std::mutex> stateLock(state_lock_); 
      mj_step2(mj_model_, mj_data_);  // step2: 推进仿真
    }
  }
}

void G1Sim2MujocoEnv::moveToDefaultPos() {
  FRC_INFO("[G1Sim2MujocoEnv.moveToDefaultPos] Moving to default position...");
  
  // 设定初始 root 位姿
  Eigen::Vector3f root_xyz(0.0f, 0.0f, 1.2f);
  Eigen::Vector4f root_rot(1.0f, 0.0f, 0.0f, 0.0f); // 无旋转
  Eigen::VectorXf joint_pos = cfg_->default_angles;  // 从 config 加载

  // 边界检查（防仿真爆炸）
  // assert((joint_pos.array() >= joint_lower.array() - 1e-3f).all());
  // assert((joint_pos.array() <= joint_upper.array() + 1e-3f).all());

  {  
    std::lock_guard<std::mutex> stateLock(state_lock_);  // 自动锁定 state
    mj_step1(mj_model_, mj_data_); // step1: 更新状态，获得最新 qpos/qvel/contact 等
  }
  {
    // 设置 qpos（位置） = [root_xyz(3) + root_rot(4) + joint_pos(n)]
    for (int i = 0; i < 3; ++i) mj_data_->qpos[i] = double(root_xyz[i]);
    for (int i = 0; i < 4; ++i) mj_data_->qpos[3 + i] = double(root_rot[i]);
    for (int i = 0; i < jointDim_; ++i) mj_data_->qpos[7 + i] = double(joint_pos[i]);
    // 清零速度和控制输入
    std::fill(mj_data_->qvel, mj_data_->qvel + gvDim_, 0.0);
    std::fill(mj_data_->ctrl, mj_data_->ctrl + jointDim_, 0.0);
  }
  {
    std::lock_guard<std::mutex> stateLock(state_lock_);  // 自动锁定 state
    mj_step2(mj_model_, mj_data_);  // step2: 推进仿真
  }

  // 初始化当前动作缓存（用于后续策略初始化）
  pTarget = Eigen::Map<Eigen::VectorXd>(mj_data_->qpos + 7, jointDim_).cast<float>();
  updateRobotState();

  std::ostringstream oss;
  for (int i = 0; i < gcDim_; ++i)
    oss << mj_data_->qpos[i] << " ";
  FRC_INFO("[G1Sim2MujocoEnv.moveToDefaultPos] Default pose: " <<oss.str());
  FRC_INFO("[G1Sim2MujocoEnv.moveToDefaultPos] Default pose initialized.");
}

void G1Sim2MujocoEnv::stop(){
  running_ = false;
}

G1Sim2MujocoEnv::~G1Sim2MujocoEnv() {
  if (mj_data_) mj_deleteData(mj_data_);
  if (mj_model_) mj_deleteModel(mj_model_);
  mjv_freeScene(&scn_);
  mjr_freeContext(&con_);
  if (!headless_) {
      if (window_) glfwDestroyWindow(window_);
      glfwTerminate();
  }
}

