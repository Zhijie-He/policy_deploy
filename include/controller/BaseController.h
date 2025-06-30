#pragma once
#include <unordered_map>
#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <atomic>
#include <Eigen/Core>
#include <vector>
#include <torch/torch.h>
#include "tasks/BaseTask.h"
#include "config/BaseRobotConfig.h"
#include "hardware/listener.h"

class BaseController {
public:
    BaseController(const std::vector<std::pair<std::string, char>>& registers,
                   std::shared_ptr<BaseRobotConfig> cfg,
                   torch::Device device);
                   
    virtual ~BaseController();
    virtual void zero_torque_state() = 0;
    virtual void move_to_default_pose() = 0;
    virtual void default_pos_state() = 0;
    virtual void run() = 0;

protected:
    float control_dt_;
    std::shared_ptr<BaseRobotConfig> cfg_ = nullptr;
    std::string current_task_;
    std::vector<std::string> task_name_list_;
    std::unordered_map<std::string, std::shared_ptr<BaseTask>> tasks_;
    std::unordered_map<std::string, char> keyboards_;

    std::mutex register_mutex_;
    std::thread keyboard_thread_;
    std::atomic<bool> keyboard_thread_running_{true};
};

