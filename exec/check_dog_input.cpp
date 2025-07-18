#include <Eigen/Dense>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <random>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
using namespace eprosima::fastdds::dds;

#include "wlrobot_dds/hd/IMUStatePubSubTypes.hpp"
#include "wlrobot_dds/hd/MotorStatesPubSubTypes.hpp"
using namespace wlrobot::msg;

#include "utility/tools.h"
#include "utility/timer.h"
#include "utility/cxxopts.hpp"
#define LOG_USE_COLOR 1
#define LOG_USE_PREFIX 1
#include "utility/logger.h"

// global low state
IMUState latestImuState;
MotorStates latestMotorStates;
std::mutex imuStateMutex;
std::mutex motorStatesMutex;

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
                IMUState imu;
                MotorStates motor_state;
                {
                    std::lock_guard<std::mutex> lock(imuStateMutex);
                    imu = latestImuState;
                }
                {
                    std::lock_guard<std::mutex> lock(motorStatesMutex);
                    motor_state = latestMotorStates;
                }
     
                int joint_num = std::min(int(motor_state.w().size()), jointDim_);

                // root xyz
                mj_data_->qpos[0] = 0;
                mj_data_->qpos[1] = 0;
                mj_data_->qpos[2] = 1.2;

                // 写入 base 姿态（IMU → 四元数）
                Eigen::AngleAxisf rollAngle(imu.rpy()[0], Eigen::Vector3f::UnitX());
                Eigen::AngleAxisf pitchAngle(imu.rpy()[1], Eigen::Vector3f::UnitY());
                Eigen::AngleAxisf yawAngle(imu.rpy()[2], Eigen::Vector3f::UnitZ());

                Eigen::Quaternionf quat = yawAngle * pitchAngle * rollAngle;  // ZYX顺序
               
                // mj_data_->qpos[3] = quat.w();
                // mj_data_->qpos[4] = quat.x();
                // mj_data_->qpos[5] = quat.y();
                // mj_data_->qpos[6] = quat.z();
                mj_data_->qpos[3] = imu.quaternion()[0];
                mj_data_->qpos[4] = imu.quaternion()[1];
                mj_data_->qpos[5] = imu.quaternion()[2];
                mj_data_->qpos[6] = imu.quaternion()[3];

                // 写入 base angular velocity（IMU gyroscope → base twist 的旋转部分）
                mj_data_->qvel[3] = imu.gyroscope()[0];  // ωx
                mj_data_->qvel[4] = imu.gyroscope()[1];  // ωy
                mj_data_->qvel[5] = imu.gyroscope()[2];  // ωz
                
                // 写入关节数据
                for (int i = 0; i < joint_num; ++i) {
                    mj_data_->qpos[7 + i] = motor_state.pos()[i];
                    mj_data_->qvel[6 + i] = motor_state.w()[i];
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

class DDSSubscriberManager {
public:
    bool init() {
        DomainParticipantQos participantQos;
        participantQos.name("MujocoViewerParticipant");

        participant_ = DomainParticipantFactory::get_instance()->create_participant(1, participantQos);
        if (!participant_) return false;

        subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
        if (!subscriber_) return false;

        imuType_ = TypeSupport(new IMUStatePubSubType());
        motorType_ = TypeSupport(new MotorStatesPubSubType());

        imuType_.register_type(participant_);
        motorType_.register_type(participant_);

        Topic* imuTopic = participant_->create_topic("/low_level/imu/state", "wlrobot::msg::IMUState", TOPIC_QOS_DEFAULT);
        Topic* motorTopic = participant_->create_topic("/low_level/motor/state", "wlrobot::msg::MotorStates", TOPIC_QOS_DEFAULT);
        if (!imuTopic || !motorTopic) return false;

        imuReader_ = subscriber_->create_datareader(imuTopic, DATAREADER_QOS_DEFAULT, &imuListener_);
        motorReader_ = subscriber_->create_datareader(motorTopic, DATAREADER_QOS_DEFAULT, &motorListener_);

        return imuReader_ && motorReader_;
    }

    void shutdown() {
        if (participant_) {
            DomainParticipantFactory::get_instance()->delete_participant(participant_);
            participant_ = nullptr;
        }
    }

private:
    DomainParticipant* participant_ = nullptr;
    Subscriber* subscriber_ = nullptr;
    DataReader* imuReader_ = nullptr;
    DataReader* motorReader_ = nullptr;
    TypeSupport imuType_;
    TypeSupport motorType_;

    class IMUListener : public DataReaderListener {
    public:
        void on_data_available(DataReader* reader) override {
            SampleInfo info;
            IMUState msg;
            if (reader->take_next_sample(&msg, &info) == eprosima::fastdds::dds::RETCODE_OK) {
                std::lock_guard<std::mutex> lock(imuStateMutex);
                latestImuState = msg;
            }
        }

        void on_subscription_matched(DataReader*, const SubscriptionMatchedStatus& info) override
        {
            if (info.current_count_change == 1)
            {
                std::cout << "IMUListener Subscriber matched." << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                std::cout << "IMUListener Subscriber unmatched." << std::endl;
            }
            else
            {
                std::cout << info.current_count_change
                          << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
            }
        }
    };

    class MotorListener : public DataReaderListener {
    public:
        MotorListener() {
            abad_sign  = { 1.0f,  1.0f, -1.0f, -1.0f };
            hip_sign   = {-1.0f,  1.0f, -1.0f,  1.0f };
            knee_sign  = {-1.0f,  1.0f, -1.0f,  1.0f };

            real2sim_dof_map = {
                3, 4, 5,
                0, 1, 2,
                9,10,11,
                6, 7, 8
            };

            zero_offset = {{
                { 0.882785f,  1.56933f,  -3.15066f },
                {-0.245677f, -0.873517f,  2.25232f },
                { 0.169374f,  1.31086f,  -2.8007f  },
                { 0.422843f, -0.535587f,  2.45248f }
            }};
        }

        void on_data_available(DataReader* reader) override {
            SampleInfo info;
            MotorStates msg;

            if (reader->take_next_sample(&msg, &info) == eprosima::fastdds::dds::RETCODE_OK) {
                std::lock_guard<std::mutex> lock(motorStatesMutex);

                // 实际读取字段
                const auto& pos_in  = msg.pos();
                const auto& vel_in  = msg.w();
                const auto& tau_in  = msg.t();
                const auto& temp_in = msg.temperature();
                const auto& mode_in = msg.mode();

                // 中间变量：在 real 顺序上先处理好 offset + sign
                float real_pos[12];
                float real_vel[12];
                float real_tau[12];
                int   real_temp[12];
                int   real_mode[12];

                for (int i = 0; i < 12; ++i) {
                    int leg = i / 3;
                    int joint = i % 3;

                    float sign = (joint == 0) ? abad_sign[leg]
                                : (joint == 1) ? hip_sign[leg]
                                            : knee_sign[leg];
                    float offset = zero_offset[leg][joint];

                    real_pos[i]  = (pos_in[i] - offset) * sign;
                    real_vel[i]  = vel_in[i] * sign;
                    real_tau[i]  = tau_in[i] * sign;
                    real_temp[i] = temp_in[i];
                    real_mode[i] = mode_in[i];
                }

                // 输出字段
                auto& pos_out  = latestMotorStates.pos();
                auto& vel_out  = latestMotorStates.w();
                auto& tau_out  = latestMotorStates.t();
                auto& temp_out = latestMotorStates.temperature();
                auto& mode_out = latestMotorStates.mode();

                for (int sim_idx = 0; sim_idx < 12; ++sim_idx) {
                    int real_idx = real2sim_dof_map[sim_idx];

                    pos_out[sim_idx]  = real_pos[real_idx];
                    vel_out[sim_idx]  = real_vel[real_idx];
                    tau_out[sim_idx]  = real_tau[real_idx];
                    temp_out[sim_idx] = real_temp[real_idx];
                    mode_out[sim_idx] = real_mode[real_idx];
                }
            }
        }
            
        void on_subscription_matched(DataReader*, const SubscriptionMatchedStatus& info) override
        {
            if (info.current_count_change == 1)
            {
                std::cout << "MotorListener Subscriber matched." << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                std::cout << "MotorListener Subscriber unmatched." << std::endl;
            }
            else
            {
                std::cout << info.current_count_change
                          << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
            }
        }

    private:
        std::array<float, 4> abad_sign;
        std::array<float, 4> hip_sign;
        std::array<float, 4> knee_sign;
        std::array<int, 12> real2sim_dof_map;
        std::array<std::array<float, 3>, 4> zero_offset;
    };


    IMUListener imuListener_;
    MotorListener motorListener_;
};


std::shared_ptr<BaseRobotConfig> cfg = nullptr;


int main(int argc, char** argv) {
    cxxopts::Options options("check_dog_input", "Check robot input and visualize using Mujoco viewer");
    options.add_options()
        ("c,config", "Config name (e.g., mdl)", cxxopts::value<std::string>())
        ("h,help", "Print usage");

    auto result = options.parse(argc, argv);
    if (result.count("help") || !result.count("config")) {
        std::cout << options.help() << std::endl;
        return 0;
    }
    std::string config_name = result["config"].as<std::string>();

    DDSSubscriberManager dds_sub_mgr;
    if (!dds_sub_mgr.init()) {
        FRC_ERROR("Failed to initialize DDSSubscriberManager");
        return 1;
    }

    try {
      cfg = tools::loadConfig(config_name);
      MinimalMujocoViewer viewer(cfg);
      std::thread integrate_thread(&MinimalMujocoViewer::integrate, &viewer);
      viewer.renderLoop();
      integrate_thread.join();
    } catch (const std::exception& e) {
      FRC_ERROR("Exception: " << e.what());
      return 1;
    }
    return 0;
}
