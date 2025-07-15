// ===== src/sim2/BaseEnv.cpp =====
#include "sim2/base_env.h"
#include "utility/timer.h"

BaseEnv::BaseEnv(std::shared_ptr<const BaseRobotConfig> cfg,
                 const std::string& hands_type,
                 std::shared_ptr<StateMachine> state_machine)
    :cfg_(cfg),
     hands_type_(hands_type),
     robotName_(cfg->robot_name),
     jointDim_(cfg_->num_actions),
     handsDim_(cfg->hand_map.at(hands_type).hands_num),
     control_dt_(cfg->getPolicyDt()),
     state_machine_(state_machine),
     jointCMDBufferPtr_(state_machine->getJointCMDBufferPtr()),
     robotStatusBufferPtr_(state_machine_->getRobotStatusBufferPtr())
{
    if (cfg_->hand_map.find(hands_type_) == cfg_->hand_map.end()) {
        throw std::runtime_error("[BaseEnv.Const] Unknown hands_type: " + hands_type_);
    }
}

void BaseEnv::initWorld() {
    actuatorDim_ = jointDim_ + handsDim_; 
    gcDim_ = actuatorDim_ + 7;
    gvDim_ = actuatorDim_ + 6;

    // 加载手部控制配置
    const auto& hand_cfg = cfg_->hand_map.at(hands_type_);
    left_hand_num_dof_   = hand_cfg.left_hand_num_dof;
    right_hand_num_dof_  = hand_cfg.right_hand_num_dof;
    joint_concat_index_  = hand_cfg.joint_concat_index;
    joint_split_index_   = hand_cfg.joint_split_index;

    // print info
    FRC_INFO("[BaseEnv.initWorld] Robot name: " << robotName_);
    FRC_INFO("[BaseEnv.initWorld] GC (qpos) Dim: " << gcDim_);
    FRC_INFO("[BaseEnv.initWorld] GV (qvel) Dim: " << gvDim_);
    FRC_INFO("[BaseEnv.initWorld] Actuator Dim: " << actuatorDim_);
    FRC_INFO("[BaseEnv.initWorld] Joint Dim: " << jointDim_ << ", Hands Dim: " << handsDim_);
    FRC_HIGHLIGHT("[BaseEnv.initWorld] Hands type: " << hands_type_);
    
    initState();
}

void BaseEnv::initState() {
    // ① 机器人状态变量
    gc_.setZero(gcDim_);                //  当前 generalized coordinate（广义坐标，位置）
    gv_.setZero(gvDim_);                //  当前 generalized velocity（广义速度）
    joint_torques_.setZero(actuatorDim_);  //  当前 tau
    
    // ② 控制增益 P and D Gain
    jointPGain      = cfg_->kP;
    leftHandPGain   = cfg_->hand_map.at(hands_type_).kp_left;
    rightHandPGain  = cfg_->hand_map.at(hands_type_).kp_right;
    Eigen::VectorXf raw_kp(jointPGain.size() + leftHandPGain.size() + rightHandPGain.size());
    raw_kp << jointPGain, leftHandPGain, rightHandPGain;
    fullPGain = tools::resolveCompatibilityConcat(raw_kp, joint_concat_index_);
    
    jointDGain      = cfg_->kD;
    leftHandDGain   = cfg_->hand_map.at(hands_type_).kd_left;
    rightHandDGain  = cfg_->hand_map.at(hands_type_).kd_right;
    Eigen::VectorXf raw_kd(jointDGain.size() + leftHandDGain.size() + rightHandDGain.size());
    raw_kd << jointDGain, leftHandDGain, rightHandDGain;
    fullDGain = tools::resolveCompatibilityConcat(raw_kd, joint_concat_index_);

    // ③ 动作目标
    pTarget.setZero(actuatorDim_); // desired position（用于位置控制）
    vTarget.setZero(actuatorDim_); // desired velocity（用于速度控制）

    // ④ 力矩命令
    tauCmd.setZero(actuatorDim_); //	最终输出的控制力矩（全体）
}

void BaseEnv::runControlLoop() {
    RateLimiter controlTimer(1.0 / control_dt_, "main loop");

    int print_interval = 300;
    while (isRunning()) {
        // == 状态机 step ==
        if (state_machine_) {
            auto t_start = std::chrono::high_resolution_clock::now();
            state_machine_->step();
            auto t_end = std::chrono::high_resolution_clock::now();

            double dt = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            run_sum_us += dt;
            run_sum_sq_us += dt * dt;
            ++run_count;

            if (run_count % print_interval == 0) {
                double avg = run_sum_us / run_count;
                double stddev = std::sqrt(run_sum_sq_us / run_count - avg * avg);
                FRC_INFO("[StateMachine.step] " << print_interval << " steps AVG: " << avg << " ms | STDDEV: " << stddev << " ms");
                // reset
                run_sum_us = run_sum_sq_us = run_count = 0;
            }
        }

        // == 等待动作 ==
        auto actionPtr = jointCMDBufferPtr_->GetData();
        while (!actionPtr) {
            FRC_INFO("[BaseEnv.runControlLoop] Waiting For actions!");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            actionPtr = jointCMDBufferPtr_->GetData();
        }

        {
            std::lock_guard<std::mutex> lock(action_lock_);
            const auto& cmd = *actionPtr;
            applyAction(cmd);  // 委托子类执行
        }
        controlTimer.wait();
    }
}

