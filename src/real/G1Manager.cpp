// G1Manager.cpp
#include "real/G1Manager.h"
#include "utility/logger.h"
#include "utility/timer.h"
#include "utility/orientation_tools.h"
#include <cstring>
#include <cmath>
#include <thread>
#include "utility/pd_control.h"

G1Manager::G1Manager(std::shared_ptr<const BaseRobotConfig> cfg,
                    std::shared_ptr<jointCMD> jointCMDPtr,
                    std::shared_ptr<robotStatus> robotStatusPtr)
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
  initState();
  updateRobotState();
  launchServer();
  FRC_INFO("[MjcMgr.Const] Ready.");
}

void G1Manager::initWorld() {
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

  FRC_INFO("[MjcMgr.initWorld] Initial qpos from XML:");
  std::ostringstream oss;
  for (int i = 0; i < gcDim_; ++i)
    oss << mj_data_->qpos[i] << " ";
  FRC_INFO("[MjcMgr.initWorld]: XML Pos " <<oss.str());
}

void G1Manager::initState() {
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

void G1Manager::updateRobotState() {
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

void G1Manager::launchServer() {
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

    auto* mgr = static_cast<G1Manager*>(glfwGetWindowUserPointer(window));
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
    auto* mgr = static_cast<G1Manager*>(glfwGetWindowUserPointer(window));
    if (!mgr) return;
    mjv_moveCamera(mgr->mj_model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &mgr->scn_, &mgr->cam_);
  });

  FRC_INFO("[MjcMgr.launchServer] GLFW Initialized");
  FRC_INFO("[MjcMgr.launchServer] GLFW Window: " << window_);
}

void G1Manager::run() {
  Timer controlTimer(control_dt_);
  while (!glfwWindowShouldClose(window_) && running_) {
    auto action = std::make_shared<jointCMD>();

    {
      std::lock_guard<std::mutex> actionLock(action_lock_);
      memcpy(action.get(), jointCMDPtr_.get(), sizeof(jointCMD));
      memcpy(pTarget.data(), action->data.position, sizeof(float) * jointDim_);
      memcpy(vTarget.data(), action->data.velocity, sizeof(float) * jointDim_);
      memcpy(jointPGain.data(), action->data.kp, sizeof(float) * jointDim_);
      memcpy(jointDGain.data(), action->data.kd, sizeof(float) * jointDim_);
    }

    if (isStatesReady) updateRobotState();
    controlTimer.wait();
  }
}

void G1Manager::integrate() {
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

void G1Manager::renderLoop() {
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

G1Manager::~G1Manager() {
  if (mj_data_) mj_deleteData(mj_data_);
  if (mj_model_) mj_deleteModel(mj_model_);
  mjv_freeScene(&scn_);
  mjr_freeContext(&con_);
  if (window_) glfwDestroyWindow(window_);
  glfwTerminate();
}

