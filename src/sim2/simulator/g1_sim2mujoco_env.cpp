// G1Sim2MujocoEnv.cpp
#include <cstring>
#include <cmath>
#include <thread>
#include "sim2/simulator/g1_sim2mujoco_env.h"
#include "utility/logger.h"
#include "utility/timer.h"
#include "utility/tools.h"

G1Sim2MujocoEnv::G1Sim2MujocoEnv(std::shared_ptr<const BaseRobotConfig> cfg,
                                 const std::string& hands_type,
                                 std::shared_ptr<StateMachine> state_machine)
    : BaseEnv(cfg, hands_type, state_machine),
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
  if(hands_type_ == "dex3"){
    mj_model_ = mj_loadXML(cfg_->xml_with_hand_path.c_str(), nullptr, error, 1000);
  }else{
    mj_model_ = mj_loadXML(cfg_->xml_path.c_str(), nullptr, error, 1000);
  }

  if (!mj_model_) {
    FRC_ERROR("[G1Sim2MujocoEnv.initWorld] Failed to load "<< cfg_->xml_path.c_str() << ", " << error);
    std::exit(1);
  }
  
  mj_data_ = mj_makeData(mj_model_);
  mj_model_->opt.timestep = simulation_dt_;

  gcDim_        = mj_model_->nq;  // generalized coordinates (qpos)
  gvDim_        = mj_model_->nv;  // generalized velocities (qvel)
  actuatorDim_  = mj_model_->nu;  // number of actuators = ctrl dimension

  const int expected_actuator_dim = jointDim_ + handsDim_;

  if (actuatorDim_ != expected_actuator_dim) {
      std::ostringstream oss;
      oss << "[G1Sim2MujocoEnv] Actuator dimension mismatch!\n"
          << "  mj_model_->nu             = " << actuatorDim_ << "\n"
          << "  expected (joint + hands)  = " << expected_actuator_dim << " ("
          << jointDim_ << " + " << handsDim_ << ")\n"
          << "  --> Please check XML <actuator> block for completeness.";

      throw std::runtime_error(oss.str());
  }

  FRC_INFO("[G1Sim2MujocoEnv.initWorld] Loaded robot: " << robotName_ << " with XML: " << cfg_->xml_path.c_str());
  FRC_INFO("[G1Sim2MujocoEnv.initWorld] generalize Coordinate dimensions: " << gcDim_);
  FRC_INFO("[G1Sim2MujocoEnv.initWorld] generalize Vel dimensions: " << gvDim_);
  FRC_INFO("[G1Sim2MujocoEnv.initWorld] Joint Num: " << jointDim_);
  FRC_INFO("[G1Sim2MujocoEnv.initWorld] hands Num: " << handsDim_);

  std::ostringstream oss;
  for (int i = 0; i < gcDim_; ++i)
    oss << mj_data_->qpos[i] << " ";
  FRC_INFO("[G1Sim2MujocoEnv.initWorld] Initial qpos from XML: " <<oss.str());

  // add hands related value
  left_hand_num_dof_  = cfg_->hand_map.at(hands_type_).left_hand_num_dof;
  right_hand_num_dof_ = cfg_->hand_map.at(hands_type_).right_hand_num_dof;
  joint_concat_index_ = cfg_->hand_map.at(hands_type_).joint_concat_index;
  joint_split_index_  = cfg_->hand_map.at(hands_type_).joint_split_index;

  // FRC_CRITICAL("hand num " << left_hand_num_dof_ << " " << right_hand_num_dof_);
  // FRC_CRITICAL("joint_concat_index_ " << joint_concat_index_.transpose());
  // FRC_CRITICAL("joint_split_index_skeleton " << joint_split_index_.at("skeleton").transpose());
  // FRC_CRITICAL("joint_split_index_hands " << joint_split_index_.at("hands").transpose());

  FRC_INFO("[G1Sim2MujocoEnv.initWorld] Add hands logical");
}

void G1Sim2MujocoEnv::initState() {
  // ① 机器人状态变量
  gc_.setZero(gcDim_);                //  当前 generalized coordinate（广义坐标，位置）
  gv_.setZero(gvDim_);                //  当前 generalized velocity（广义速度）
  joint_torques_.setZero(actuatorDim_);  //  当前 tau

  // ② 控制增益
  // jointPGain = cfg_->kP;
  // jointDGain = cfg_->kD;
  {
    std::vector<float> kp_vec(cfg_->kP.data(), cfg_->kP.data() + cfg_->kP.size());
    const auto& kp_left  = cfg_->hand_map.at(hands_type_).kp_left;
    const auto& kp_right = cfg_->hand_map.at(hands_type_).kp_right;
    kp_vec.insert(kp_vec.end(), kp_left.begin(), kp_left.end());
    kp_vec.insert(kp_vec.end(), kp_right.begin(), kp_right.end());
    Eigen::VectorXf raw_kp = Eigen::Map<Eigen::VectorXf>(kp_vec.data(), kp_vec.size());
    jointPGain = tools::resolveCompatibilityConcat(raw_kp, joint_concat_index_);
  }
  // FRC_CRITICAL("jointPGain " << jointPGain.transpose());
  {
    std::vector<float> kd_vec(cfg_->kD.data(), cfg_->kD.data() + cfg_->kD.size());
    const auto& kp_left  = cfg_->hand_map.at(hands_type_).kp_left;
    const auto& kd_left  = cfg_->hand_map.at(hands_type_).kd_left;
    const auto& kd_right = cfg_->hand_map.at(hands_type_).kd_right;
    kd_vec.insert(kd_vec.end(), kd_left.begin(), kd_left.end());
    kd_vec.insert(kd_vec.end(), kd_right.begin(), kd_right.end());
    Eigen::VectorXf raw_kd = Eigen::Map<Eigen::VectorXf>(kd_vec.data(), kd_vec.size());
    jointDGain = tools::resolveCompatibilityConcat(raw_kd, joint_concat_index_);
  }
  // FRC_CRITICAL("jointDGain " << jointDGain.transpose());
  
  // ③ 动作目标
  pTarget.setZero(actuatorDim_); // desired position（用于位置控制）
  vTarget.setZero(actuatorDim_); // desired velocity（用于速度控制）

  // ④ 力矩命令
  tauCmd.setZero(actuatorDim_); //	最终输出的控制力矩（全体）
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
    joint_torques_  = Eigen::Map<Eigen::VectorXd>(mj_data_->ctrl, actuatorDim_);
    positionVec = gc_.cast<float>();
    velocityVec = gv_.cast<float>();
    jointTorquesVec = joint_torques_.cast<float>();
    timestamp = mj_data_->time;
  }
  
  if (robotStatusBufferPtr_) {
    robotStatus status;
    memcpy(status.data.position, positionVec.data(), gcDim_ * sizeof(float));
    memcpy(status.data.velocity, velocityVec.data(), gvDim_ * sizeof(float));
    memcpy(status.data.jointTorques, jointTorquesVec.data(), actuatorDim_ * sizeof(float));
    status.data.timestamp = timestamp;
    robotStatusBufferPtr_->SetData(status);
  }
}

bool G1Sim2MujocoEnv::isRunning() const {
  return running_ && (headless_ || !glfwWindowShouldClose(window_));
}

void G1Sim2MujocoEnv::applyAction(const jointCMD& cmd) {
  memcpy(pTarget.data(), cmd.data.position, sizeof(float) * actuatorDim_);
  memcpy(vTarget.data(), cmd.data.velocity, sizeof(float) * actuatorDim_);
  // memcpy(jointPGain.data(), cmd.data.kp, sizeof(float) * actuatorDim_);
  // memcpy(jointDGain.data(), cmd.data.kd, sizeof(float) * actuatorDim_);
  // kp and kd need to rethink how to sync
}

void G1Sim2MujocoEnv::step() {
  runControlLoop();
}

void G1Sim2MujocoEnv::run() {
  if (headless_) {
    FRC_INFO("[G1Sim2MujocoEnv.run] Headless mode. Running step loop without rendering.");
  } else {
    launchServer();  // 初始化 GUI 窗口和渲染上下文等
  }

  step_thread_ = std::thread(&G1Sim2MujocoEnv::step, this);
  RateLimiter renderTimer(1.0 / control_dt_, "mujoco render loop", false);
  while (isRunning()) {
    integrate();  // 控制数据集成
    if (!headless_) {
      {
        std::lock_guard<std::mutex> lock(state_lock_);
        // 更新视觉场景
        mjv_updateScene(mj_model_, mj_data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
        // 渲染图像
        int width, height;
        glfwGetFramebufferSize(window_, &width, &height);
        mjr_render({0, 0, width, height}, &scn_, &con_);
      }
      glfwSwapBuffers(window_);
      glfwPollEvents();
    }
    updateRobotState();
    renderTimer.wait();  // 控制渲染频率
  }
}

void G1Sim2MujocoEnv::integrate() {
  for (int i = 0; i < int(control_dt_ / simulation_dt_); i++) {
    {   // or mjcb_control 简化结构
      std::lock_guard<std::mutex> stateLock(state_lock_);  
      mj_step1(mj_model_, mj_data_); // step1: 更新状态，获得最新 qpos/qvel/contact 等
    }
    { 
      std::lock_guard<std::mutex> actionLock(action_lock_);
      Eigen::VectorXf joint_pos = Eigen::Map<Eigen::VectorXd>(mj_data_->qpos + 7, actuatorDim_).cast<float>();
      Eigen::VectorXf joint_vel = Eigen::Map<Eigen::VectorXd>(mj_data_->qvel + 6, actuatorDim_).cast<float>();
      tauCmd = tools::pd_control(pTarget, joint_pos, jointPGain, vTarget, joint_vel, jointDGain);
      for (int i = 0; i < actuatorDim_; ++i) mj_data_->ctrl[i] = tauCmd[i];
    }
    {
      std::lock_guard<std::mutex> stateLock(state_lock_); 
      mj_step2(mj_model_, mj_data_);  // step2: 推进仿真
    }
  }
}

void G1Sim2MujocoEnv::moveToDefaultPos() {
  FRC_INFO("[G1Sim2MujocoEnv.moveToDefaultPos] Moving to default position...");

  Eigen::Vector3f start_root_xyz(0.0f, 0.0f, 0.8f);
  Eigen::Vector4f start_root_rot(1.0f, 0.0f, 0.0f, 0.0f);
  Eigen::VectorXf target_joint_pos = cfg_->default_angles;

  // 1. 如果需要扩展 hand DOF：
  if (handsDim_ > 0) {
    Eigen::VectorXf hand_vec = Eigen::VectorXf::Ones(handsDim_);
    Eigen::VectorXf extended(target_joint_pos.size() + hand_vec.size());
    extended << target_joint_pos, hand_vec;
    target_joint_pos = extended;
  }

  // 2. 兼容拼接（resolveCompatibilityConcat）
  target_joint_pos = tools::resolveCompatibilityConcat(target_joint_pos, joint_concat_index_);

  // 3. 拆分（resolveCompatibilitySplit）
  auto split_result = tools::resolveCompatibilitySplit(target_joint_pos, joint_split_index_);
  Eigen::VectorXf skeleton_joint_pos = split_result.first;  // 获取第一个部分
  
  // 边界检查（防仿真爆炸） 
  // assert((target_joint_pos.array() >= joint_lower.array() - 1e-3f).all());
  // assert((target_joint_pos.array() <= joint_upper.array() + 1e-3f).all());

  {  
    std::lock_guard<std::mutex> stateLock(state_lock_);  // 自动锁定 state
    mj_step1(mj_model_, mj_data_); // step1: 更新状态，获得最新 qpos/qvel/contact 等
  }
  {
    // 设置 qpos（位置） = [start_root_xyz(3) + start_root_rot(4) + target_joint_pos(n)]
    for (int i = 0; i < 3; ++i) mj_data_->qpos[i] = double(start_root_xyz[i]);
    for (int i = 0; i < 4; ++i) mj_data_->qpos[3 + i] = double(start_root_rot[i]);
    for (int i = 0; i < actuatorDim_; ++i) mj_data_->qpos[7 + i] = double(target_joint_pos[i]);

    // 清零速度和控制输入
    std::fill(mj_data_->qvel, mj_data_->qvel + gvDim_, 0.0);
    std::fill(mj_data_->ctrl, mj_data_->ctrl + actuatorDim_, 0.0);
  }
  {
    std::lock_guard<std::mutex> stateLock(state_lock_);  // 自动锁定 state
    mj_step2(mj_model_, mj_data_);  // step2: 推进仿真
  }

  // 初始化当前动作缓存（用于后续策略初始化）
  pTarget = Eigen::Map<Eigen::VectorXd>(mj_data_->qpos + 7, actuatorDim_).cast<float>();
  updateRobotState();

  std::ostringstream oss;
  for (int i = 0; i < gcDim_; ++i)
    oss << mj_data_->qpos[i] << " ";
  FRC_INFO("[G1Sim2MujocoEnv.moveToDefaultPos] Default pose: " <<oss.str());
  FRC_INFO("[G1Sim2MujocoEnv.moveToDefaultPos] Default pose initialized.");
}

void G1Sim2MujocoEnv::stop() {
  running_ = false;
  if (step_thread_.joinable()) {
    step_thread_.join(); 
  }
}

G1Sim2MujocoEnv::~G1Sim2MujocoEnv() {
  if (mj_data_) mj_deleteData(mj_data_);
  if (mj_model_) mj_deleteModel(mj_model_);

  if (!headless_) {
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);

    // 安全销毁 window
    if (window_ && !glfwWindowShouldClose(window_)) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
  }
}
