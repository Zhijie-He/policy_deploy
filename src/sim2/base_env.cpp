// G1Sim2MujocoEnv.cpp
#include "sim2/base_env.h"
#include "utility/timer.h"

BaseEnv::BaseEnv(std::shared_ptr<const BaseRobotConfig> cfg,
          std::shared_ptr<StateMachine> state_machine)
    : cfg_(cfg),
      state_machine_(state_machine),
      jointCMDBufferPtr_(state_machine->getJointCMDBufferPtr()),
      robotStatusBufferPtr_(state_machine_->getRobotStatusBufferPtr()),
      control_dt_(cfg->getPolicyDt())
{   

}

void BaseEnv::initState() {
  // ① 机器人状态变量
  gc_.setZero(gcDim_);                //  当前 generalized coordinate（广义坐标，位置）
  gv_.setZero(gvDim_);                //  当前 generalized velocity（广义速度）
  joint_torques_.setZero(jointDim_);  //  当前 tau

  // ② 控制增益
  jointPGain = cfg_->kP;
  jointDGain = cfg_->kD;
  
  // ③ 动作目标
  pTarget.setZero(jointDim_); // desired position（用于位置控制）
  vTarget.setZero(jointDim_); // desired velocity（用于速度控制）

  // ④ 力矩命令
  tauCmd.setZero(jointDim_); //	最终输出的控制力矩（全体）
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

