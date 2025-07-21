// ===== src/sim2/BaseEnv.cpp =====
#include "sim2/base_env.h"

 BaseEnv::BaseEnv(std::shared_ptr<const BaseRobotConfig> cfg,
                  std::shared_ptr<StateMachine> state_machine)
      : cfg_(cfg),
        state_machine_(state_machine),
        jointCMDBufferPtr_(state_machine->getJointCMDBufferPtr()),
        robotStatusBufferPtr_(state_machine->getRobotStatusBufferPtr()),
        control_dt_(cfg->getPolicyDt())
{

}

void BaseEnv::initState() {
  gcDim_       = cfg_->num_actions + 7; 
  gvDim_       = cfg_->num_actions + 6; 
  jointDim_    = cfg_->num_actions;
  actuatorDim_ = jointDim_; 

  // ① 机器人状态变量
  gc_.setZero(gcDim_);      
  gv_.setZero(gvDim_);     

  // ② 控制增益
  jointPGain = cfg_->kP;
  jointDGain = cfg_->kD;

  // ③ 动作目标
  pTarget.setZero(actuatorDim_);
  vTarget.setZero(actuatorDim_); 

  // ④ 力矩命令
  tauCmd.setZero(actuatorDim_); 
}

void BaseEnv::runControlLoop() {
    RateLimiter controlTimer(1.0 / control_dt_, "main loop");

    int print_interval = cfg_->print_interval;
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

