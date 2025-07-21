#include <GLFW/glfw3.h>
#include <thread>
#include <mutex>
#include <memory>
#include <string>
#include <sstream>
#include <Eigen/Dense>

#include <wlrobot/robot/channel/channel_factory.hpp>
#include <wlrobot/robot/channel/channel_publisher.hpp>
#include <wlrobot/idl/hd/IMUStatePubSubTypes.hpp>
#include <wlrobot/idl/hd/MotorStatesPubSubTypes.hpp>
#include <wlrobot/idl/hd/MotorCmdsPubSubTypes.hpp>
#include <wlrobot/robot/low_level/low_state_aggregator.hpp>
using namespace wlrobot::robot;
using namespace wlrobot::msg;

#include "utility/tools.h"
#include "utility/timer.h"
#include "utility/logger.h"
#include "utility/cxxopts.hpp"

class MujocoPublisher {
public:
    MujocoPublisher(std::shared_ptr<BaseRobotConfig> cfg)
        : cfg_(cfg), 
          simulation_dt_(cfg->simulation_dt), 
          control_dt_(cfg->getPolicyDt())
    {
        tools::checkMujucoVersion();
        initWorld();
        initState();
        initPublisher();
    }

    void initWorld() {
        char error[1000] = "";
        mj_model_ = mj_loadXML(cfg_->xml_path.c_str(), nullptr, error, 1000);
        if (!mj_model_) {
            FRC_ERROR("[MujocoPublisher] Failed to load XML: " << error);
            std::exit(1);
        }
        mj_data_ = mj_makeData(mj_model_);
        mj_model_->opt.timestep = simulation_dt_;
        gcDim_ = mj_model_->nq;
        gvDim_ = mj_model_->nv;
        jointDim_ = mj_model_->nu;
        actuatorDim_ = jointDim_;
        
    }

    void initState() {
        // ① 机器人状态变量
        gc_.setZero(gcDim_);      //  当前 generalized coordinate（广义坐标，位置）
        gv_.setZero(gvDim_);      //  当前 generalized velocity（广义速度）

        jointPos.setZero(jointDim_);
        jointVel.setZero(jointDim_);

        // ② 控制增益
        jointPGain.setZero(jointDim_); 
        jointDGain.setZero(jointDim_); 

        // ③ 动作目标
        pTarget.setZero(jointDim_); // desired position（用于位置控制）
        vTarget.setZero(jointDim_); // desired velocity（用于速度控制）

        // ④ 力矩命令
        tauCmd.setZero(jointDim_); //	最终输出的控制力矩（全体）
    }

    void initPublisher() {
        ChannelFactory::Instance()->Init(cfg_->domain_id);
        imu_pub_ = std::make_unique<ChannelPublisher<IMUState>>(cfg_->imu_topic);
        imu_pub_->InitChannel();

        motor_pub_ = std::make_unique<ChannelPublisher<MotorStates>>(cfg_->motor_topic);
        motor_pub_->InitChannel();

        lowcmd_sub_ = std::make_unique<ChannelSubscriber<MotorCmds>>(cfg_->lowcmd_topic);
        lowcmd_sub_->InitChannel(
            [this](const MotorCmds& msg) {
            this->LowCmdHandler(msg);
            }, 10
        );
    }

    // void LowCmdHandler(const MotorCmds& message) {
    //     std::lock_guard<std::mutex> actionLock(action_lock_);
    //     pTarget    = Eigen::Map<const Eigen::VectorXf>(message.pos().data(), jointDim_);
    //     vTarget    = Eigen::Map<const Eigen::VectorXf>(message.w().data(), jointDim_);
    //     jointPGain = Eigen::Map<const Eigen::VectorXf>(message.kp().data(), jointDim_);
    //     jointDGain = Eigen::Map<const Eigen::VectorXf>(message.kd().data(), jointDim_);
    // }
    
    void LowCmdHandler(const MotorCmds& message) {
        std::lock_guard<std::mutex> actionLock(action_lock_);

        // Step 1: 先 real 顺序 → sim 顺序
        std::array<float, 12> pos_real{}, vel_real{}, kp_real{}, kd_real{};
        std::array<float, 12> pos_sim{}, vel_sim{}, kp_sim{}, kd_sim{};

        for (int i = 0; i < 12; ++i) {
            pos_real[i] = message.pos()[i];
            vel_real[i] = message.w()[i];
            kp_real[i]  = message.kp()[i];
            kd_real[i]  = message.kd()[i];
        }

        for (int sim_idx = 0; sim_idx < 12; ++sim_idx) {
            int real_idx = real2sim_dof_map_[sim_idx];
            pos_sim[sim_idx] = pos_real[real_idx];
            vel_sim[sim_idx] = vel_real[real_idx];
            kp_sim[sim_idx]  = kp_real[real_idx];
            kd_sim[sim_idx]  = kd_real[real_idx];
        }

        // Step 2: sim 顺序 → 做校准（仿真使用的是校准后的值）
        Eigen::VectorXf pos_calib(jointDim_), vel_calib(jointDim_);
        Eigen::VectorXf kp_vec(jointDim_), kd_vec(jointDim_);

        for (int i = 0; i < jointDim_; ++i) {
            int leg = i / 3;
            int joint = i % 3;

            float sign = (joint == 0) ? abad_sign_[leg] :
                        (joint == 1) ? hip_sign_[leg] :
                                        knee_sign_[leg];
            float offset = zero_offset_[leg][joint];

            pos_calib(i) = (pos_sim[i] - offset) * sign;
            vel_calib(i) = vel_sim[i] * sign;
            kp_vec(i)    = kp_sim[i];
            kd_vec(i)    = kd_sim[i];
        }

        // 最终赋值
        pTarget    = pos_calib;
        vTarget    = vel_calib;
        jointPGain = kp_vec;
        jointDGain = kd_vec;
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

            auto* mgr = static_cast<MujocoPublisher*>(glfwGetWindowUserPointer(window));
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
            auto* mgr = static_cast<MujocoPublisher*>(glfwGetWindowUserPointer(window));
            if (!mgr) return;
            mjv_moveCamera(mgr->mj_model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &mgr->scn_, &mgr->cam_);
        });

        FRC_INFO("[MujocoPublisher.launchServer] GLFW Initialized");
        FRC_INFO("[MujocoPublisher.launchServer] GLFW Window: " << window_);
    }

    void renderLoop() {
        launchServer();
        
        RateLimiter renderTimer(1.0 / control_dt_, "mujoco render loop", false);
        while (!glfwWindowShouldClose(window_)) {
            integrate();  
            simulatePublisher(); // publish robot imu and joint state
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
    
    void integrate() {
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

    void simulatePublisher() {
        updateData();
        publishState();
    }
    
private:
    // void updateData() {
    //     for (int i = 0; i < gcDim_; ++i) gc_(i) = mj_data_->qpos[i];
    //     for (int i = 0; i < gvDim_; ++i) gv_(i) = mj_data_->qvel[i];
    //     for (int i = 0; i < jointDim_; ++i) {
    //         jointPos(i) = mj_data_->qpos[7 + i];
    //         jointVel(i) = mj_data_->qvel[6 + i];
    //     }
    // }

    // minic real robot motor state value and order 
    void updateData() {
        for (int i = 0; i < gcDim_; ++i) gc_(i) = mj_data_->qpos[i];
        for (int i = 0; i < gvDim_; ++i) gv_(i) = mj_data_->qvel[i];

        // Step 1: 仿真顺序 → 真实顺序（只变顺序）
        std::array<float, 12> sim_pos, sim_vel;
        for (int i = 0; i < 12; ++i) {
            sim_pos[i] = mj_data_->qpos[7 + real2sim_dof_map_[i]];
            sim_vel[i] = mj_data_->qvel[6 + real2sim_dof_map_[i]];
        }

        // Step 2: 对真实顺序数据进行反校准（sign, offset）
        for (int i = 0; i < 12; ++i) {
            int leg = i / 3;
            int joint = i % 3;

            float sign = (joint == 0) ? abad_sign_[leg] :
                        (joint == 1) ? hip_sign_[leg] :
                                        knee_sign_[leg];
            float offset = zero_offset_[leg][joint];

            jointPos(i) = sim_pos[i] * sign + offset;  // 反向零位校准
            jointVel(i) = sim_vel[i] * sign;
        }
    }

    void publishState() {
        static int64_t timestamp = 0;

        IMUState imu;
        imu.timestamp(timestamp);
        imu.temperature(36.5f);
        imu.accelerometer({0.0f, 0.0f, 9.8f});
        imu.gyroscope({gv_(3), gv_(4), gv_(5)});
        imu.quaternion({gc_(3), gc_(4), gc_(5), gc_(6)});
        imu.rpy({0.1f, 0.2f, 0.3f});
        imu_pub_->Write(imu);

        MotorStates motor;
        std::array<float, 12> pos{}, vel{}, tau{};
        std::array<int, 12> temp{};
        std::array<uint32_t, 12> mode{};
        for (int i = 0; i < std::min(12, jointDim_); ++i) {
            pos[i] = jointPos(i);
            vel[i] = jointVel(i);
            tau[i] = 0.0f;
            temp[i] = 36;
            mode[i] = 0;
        }
        motor.pos(pos);
        motor.w(vel);
        motor.t(tau);
        motor.temperature(temp);
        motor.mode(mode);
        motor_pub_->Write(motor);

        ++timestamp;
    }

    std::shared_ptr<BaseRobotConfig> cfg_;
    float simulation_dt_;
    float control_dt_;
    int gcDim_, gvDim_, jointDim_, actuatorDim_;

    Eigen::VectorXf gc_, gv_;
    Eigen::VectorXf pTarget, vTarget;
    Eigen::VectorXf jointPGain, jointDGain;
    Eigen::VectorXf tauCmd;
    Eigen::VectorXf jointPos, jointVel;

    std::unique_ptr<ChannelPublisher<IMUState>> imu_pub_;
    std::unique_ptr<ChannelPublisher<MotorStates>> motor_pub_;
    std::unique_ptr<ChannelSubscriber<MotorCmds>> lowcmd_sub_;
    
    std::mutex state_lock_;
    std::mutex action_lock_;
    std::thread run_thread_;
    // MuJoCo core members
    mjModel* mj_model_ = nullptr;
    mjData* mj_data_ = nullptr;
    GLFWwindow* window_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;
        
    //related to calibartion
    std::array<float, 4> abad_sign_  = { 1.0f,  1.0f, -1.0f, -1.0f };
    std::array<float, 4> hip_sign_   = {-1.0f,  1.0f, -1.0f,  1.0f };
    std::array<float, 4> knee_sign_  = {-1.0f,  1.0f, -1.0f,  1.0f };
    std::array<std::array<float, 3>, 4> zero_offset_ = {{
        { 0.882785f,  1.56933f,  -3.15066f },
        {-0.245677f, -0.873517f,  2.25232f },
        { 0.169374f,  1.31086f,  -2.8007f  },
        { 0.422843f, -0.535587f,  2.45248f }
    }};
    std::array<int, 12> real2sim_dof_map_ = { 3,4,5, 0,1,2, 9,10,11, 6,7,8 };
};

int main(int argc, char** argv) {
    cxxopts::Options options("robot sim sync", "Sync with robot and visualize using Mujoco");
    options.add_options()
        ("c,config", "Config name (e.g., mdl)", cxxopts::value<std::string>())
        ("h,help", "Print usage");
    auto result = options.parse(argc, argv);
    if (result.count("help") || !result.count("config")) {
        FRC_INFO(options.help());
        return 0;
    }
    
    std::string config_name = result["config"].as<std::string>();
    auto cfg = tools::loadConfig(config_name);
    
    //  start lowStateAggregator (combine imu and motor)
    auto lowstate_aggregator = std::make_unique<LowStateAggregator>(
        cfg->imu_topic,
        cfg->motor_topic,
        cfg->lowstate_topic,             // define by yourself
        cfg->domain_id                  // domain id
    );
    lowstate_aggregator->Start();

    try {
        MujocoPublisher publisher(cfg);
        publisher.renderLoop();
    } catch (const std::exception& e) {
        FRC_ERROR("Exception: " << e.what());
        return 1;
    }

    return 0;
}
