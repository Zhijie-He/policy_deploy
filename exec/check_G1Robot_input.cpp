#include <Eigen/Dense>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <random>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include "utility/real/unitree_tools.h"
#include "utility/tools.h"
#include "utility/timer.h"
#include "utility/cxxopts.hpp"

using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

// global low state
LowState_ latestLowState;
std::mutex lowstate_mutex;

class MinimalMujocoViewer {
public:
    MinimalMujocoViewer(std::shared_ptr<BaseRobotConfig> cfg)
        :cfg_(cfg), 
        robotName_(cfg->robot_name),
        simulation_dt_(cfg->simulation_dt),
        control_dt_(cfg->getPolicyDt())
    {
        tools::checkMujucoVersion();
        initWorld();
        initState();
        launchServer();
    }

    void initWorld() {
        char error[1000] = "";
        mj_model_ = mj_loadXML(cfg_->xml_path.c_str(), nullptr, error, 1000);
        if (!mj_model_) {
            FRC_ERROR("[MinimalMujocoViewer.initWorld] Failed to load XML: " << error);
            std::exit(1);
        }
        mj_data_ = mj_makeData(mj_model_);
        mj_model_->opt.timestep = simulation_dt_;

        // 取消重力
        mju_zero3(mj_model_->opt.gravity);

        gcDim_ = mj_model_->nq; // nq 是 广义坐标（generalized coordinates） 的维度。
        gvDim_ = mj_model_->nv; // nv 是 广义速度（generalized velocities） 的维度。
        jointDim_ = mj_model_->nu; // nu 是 控制输入（actuator）的数量，即 ctrl 数组的长度。

        FRC_INFO("[MinimalMujocoViewer.initWorld] Loaded robot: " << robotName_ << " with XML: " << cfg_->xml_path.c_str());
        FRC_INFO("[MinimalMujocoViewer.initWorld] generalize Coordinate dimensions: " << gcDim_);
        FRC_INFO("[MinimalMujocoViewer.initWorld] generalize Vel dimensions: " << gvDim_);
        FRC_INFO("[MinimalMujocoViewer.initWorld] Joint Num: " << jointDim_);
        FRC_INFO("[MinimalMujocoViewer.initWorld] Initial qpos from XML:");
        std::ostringstream oss;
        for (int i = 0; i < gcDim_; ++i)
            oss << mj_data_->qpos[i] << " ";
        FRC_INFO("[MinimalMujocoViewer.initWorld]: XML Pos " <<oss.str());
    }
    
    void initState() {
        // ① 机器人状态变量
        gc_.setZero(gcDim_);      //  当前 generalized coordinate（广义坐标，位置）
        gv_.setZero(gvDim_);      //  当前 generalized velocity（广义速度）

        // ② 控制增益
        jointPGain.setZero(jointDim_); 
        jointDGain.setZero(jointDim_); 

        // ③ 动作目标
        pTarget.setZero(jointDim_); // desired position（用于位置控制）
        vTarget.setZero(jointDim_); // desired velocity（用于速度控制）

        // ④ 力矩命令
        tauCmd.setZero(jointDim_); //	最终输出的控制力矩（全体）
    }

    void launchServer() {
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

            auto* mgr = static_cast<MinimalMujocoViewer*>(glfwGetWindowUserPointer(window));
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
            auto* mgr = static_cast<MinimalMujocoViewer*>(glfwGetWindowUserPointer(window));
            if (!mgr) return;
            mjv_moveCamera(mgr->mj_model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &mgr->scn_, &mgr->cam_);
        });

        FRC_INFO("[MinimalMujocoViewer.launchServer] GLFW Initialized");
        FRC_INFO("[MinimalMujocoViewer.launchServer] GLFW Window: " << window_);
    }
    
    void integrate() {
        Timer worldTimer(control_dt_);
        while (!glfwWindowShouldClose(window_)) {
            {
                std::lock_guard<std::mutex> lock(lowstate_mutex);

                const auto& motor_state = latestLowState.motor_state();
                const auto& imu = latestLowState.imu_state();
                int joint_num = std::min(int(motor_state.size()), jointDim_);

                // root xyz
                mj_data_->qpos[0] = 0;
                mj_data_->qpos[1] = 0;
                mj_data_->qpos[2] = 1.2;

                // 写入 base 姿态（IMU → 四元数）
                Eigen::AngleAxisf rollAngle(imu.rpy()[0], Eigen::Vector3f::UnitX());
                Eigen::AngleAxisf pitchAngle(imu.rpy()[1], Eigen::Vector3f::UnitY());
                Eigen::AngleAxisf yawAngle(imu.rpy()[2], Eigen::Vector3f::UnitZ());

                Eigen::Quaternionf quat = yawAngle * pitchAngle * rollAngle;  // ZYX顺序
               
                mj_data_->qpos[3] = quat.w();
                mj_data_->qpos[4] = quat.x();
                mj_data_->qpos[5] = quat.y();
                mj_data_->qpos[6] = quat.z();
                
                // 写入 base angular velocity（IMU gyroscope → base twist 的旋转部分）
                mj_data_->qvel[3] = imu.gyroscope()[0];  // ωx
                mj_data_->qvel[4] = imu.gyroscope()[1];  // ωy
                mj_data_->qvel[5] = imu.gyroscope()[2];  // ωz
                
                // 写入关节数据
                for (int i = 0; i < joint_num; ++i) {
                    mj_data_->qpos[7 + i] = motor_state.at(i).q();
                    mj_data_->qvel[6 + i] = motor_state.at(i).dq();
                }
            }
            worldTimer.wait();
        }
    }

    void renderLoop() {
        // 每秒渲染频率
        // const float render_dt = 1.0 / 120.0;
        Timer renderTimer(control_dt_);
        while (!glfwWindowShouldClose(window_)) {
            {
                std::lock_guard<std::mutex> lock(state_lock_);
                mj_step(mj_model_, mj_data_); 
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

    ~MinimalMujocoViewer() {
        if (mj_data_) mj_deleteData(mj_data_);
        if (mj_model_) mj_deleteModel(mj_model_);
        mjv_freeScene(&scn_);
        mjr_freeContext(&con_);
        if (window_) glfwDestroyWindow(window_);
        glfwTerminate();
    }

private:
    std::shared_ptr<BaseRobotConfig> cfg_ = nullptr;
    
    std::string robotName_;
    float simulation_dt_ = 5e-4;
    float control_dt_;
    int gcDim_, gvDim_, jointDim_;
    Eigen::VectorXd gc_, gv_;
    Eigen::VectorXf pTarget, vTarget;
    Eigen::VectorXf jointPGain, jointDGain;
    Eigen::VectorXf tauCmd;
    
    std::mutex state_lock_;
    std::mutex action_lock_;

    // MuJoCo core members
    mjModel* mj_model_ = nullptr;
    mjData* mj_data_ = nullptr;
    GLFWwindow* window_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;
};

void Handler(const void* message)
{   
    const LowState_& msg = *(const LowState_ *)message;
    if (msg.crc() != unitree_tools::Crc32Core((uint32_t *)&msg, (sizeof(LowState_) >> 2) - 1)) {
        FRC_ERROR("[Handler] CRC Error");
        return;
    }
    FRC_INFO("low_state with tick: " << msg.tick());
    {
        std::lock_guard<std::mutex> lock(lowstate_mutex);
        latestLowState = msg;
    }
}

std::shared_ptr<BaseRobotConfig> cfg = nullptr;

int main(int argc, char** argv) {
    cxxopts::Options options("check_G1Robot_input", "Check G1 robot input and visualize using Mujoco viewer");
    options.add_options()
        ("c,config", "Config name (e.g., g1_eman)", cxxopts::value<std::string>())
        ("h,help", "Print usage");

    auto result = options.parse(argc, argv);
    if (result.count("help") || !result.count("config")) {
        std::cout << options.help() << std::endl;
        return 0;
    }
    std::string config_name = result["config"].as<std::string>();

    ChannelFactory::Instance()->Init(0);
    ChannelSubscriber<LowState_> subscriber("rt/lowstate");
    subscriber.InitChannel(Handler);

    try {
      cfg = tools::loadConfig(config_name);
      MinimalMujocoViewer viewer(cfg);
      std::thread integrate_thread(&MinimalMujocoViewer::integrate, &viewer);
      viewer.renderLoop();
      integrate_thread.join();
    } catch (const std::exception& e) {
      std::cerr << "Exception: " << e.what() << std::endl;
      return 1;
    }
    return 0;
}
