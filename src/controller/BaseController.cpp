#include <algorithm>
#include "controller/BaseController.h"
#include "utility/logger.h"
#include "tasks/TaskFactory.h"
#include "hardware/conio.h"

BaseController::BaseController(
    const std::vector<std::pair<std::string, char>>& registers,
    std::shared_ptr<BaseRobotConfig> cfg,
    torch::Device device)
    :cfg_(cfg), control_dt_(cfg->getPolicyDt())
{
    for (const auto& [task_name, key] : registers) {
        // check if task_name is available
        if (!TaskFactory::exists(task_name)) {
            FRC_ERROR("[BaseController.Const] Invalid task name: " << task_name);
            auto available = TaskFactory::getAvailableTaskNames();
            std::ostringstream oss;
            oss << "[BaseController.Const] Available task names: ";
            for (size_t i = 0; i < available.size(); ++i) {
                oss << available[i];
                if (i != available.size() - 1) oss << " | ";
            }
            FRC_ERROR(oss.str());
            throw std::runtime_error("Invalid task name: " + task_name);
        }
        
        auto task = TaskFactory::create(task_name, control_dt_, device);
        tasks_[task_name] = task;
        keyboards_[task_name] = key;
        task_name_list_.push_back(task_name);
    }

    // 默认激活第一个任务
    if (!task_name_list_.empty()) {
        current_task_ = task_name_list_.front();  // 默认激活第一个任务
    } else {
        FRC_ERROR("[BaseController.Const] No tasks registered in BaseController");
        throw std::runtime_error("No tasks registered in BaseController.");
    }

    // 输出所有注册tasks
    std::ostringstream oss;
    oss << "[BaseController.Const] Available Tasks: ";
    for (const auto& r : task_name_list_) oss << r << " ";
    FRC_INFO(oss.str());
    FRC_INFO("[BaseController.Const] Current active task: " << current_task_);

    threads_.emplace_back(&BaseController::listenKeyboard, this);
}

void BaseController::listenKeyboard() {
    FRC_INFO("[BaseController.listenKeyboard] Press 'z' to exit keyboard listener.");
    while (isKeyBoardThreadRunning()) {
        if (kbhit()) {
            char c = getchar();
            if (c < 'Z' && c > 'A') c = c - 'A' + 'a';

            if (c == '\n') continue;
            key_input_ = c;
            FRC_INFO("[BaseController.listenKeyboard] Pressed Button: " << c);

            // 1. 退出键
            if (c == 'z') {
                stopKeyBoradThread();
                break;
            }

            // 2. 注册任务切换
            for (const auto& [task_name, key] : keyboards_) {
                if (c == key) {
                    {
                        std::lock_guard<std::mutex> lock(register_mutex_);
                        current_task_ = task_name;
                        FRC_INFO("[KeyEvent] Switched task to: " << current_task_);
                        tasks_[current_task_]->reset();
                    }
                    goto continue_loop;
                }
            }

            // 3. v：循环切换任务
            if (c == 'v') {
                std::lock_guard<std::mutex> lock(register_mutex_);
                auto it = std::find(task_name_list_.begin(), task_name_list_.end(), current_task_);
                if (it != task_name_list_.end()) {
                    size_t idx = (std::distance(task_name_list_.begin(), it) + 1) % task_name_list_.size();
                    current_task_ = task_name_list_[idx];
                    FRC_INFO("[KeyEvent] Cycled to task: " << current_task_);
                    tasks_[current_task_]->reset();
                }
                goto continue_loop;
            }

            // 4. 交由当前任务处理按键
            {
                std::lock_guard<std::mutex> lock(register_mutex_);
                if (!current_task_.empty() && tasks_.count(current_task_)) {
                    tasks_[current_task_]->resolveKeyboardInput(c);
                }
            }

            continue_loop: ;
        } else {
            usleep(10000);
        }
    }
    FRC_INFO("[BaseController.listenKeyboard] Keyboard thread exited.");
}


BaseController::~BaseController() {
    stopKeyBoradThread();  // 设置 running_ = false，让线程退出 while 循环
    for (auto& t : threads_) {
        if (t.joinable()) t.join();  // 等待线程安全退出
    }
}


// Eigen::VectorXf BaseController::getAction(const Eigen::VectorXf& self_obs, const Eigen::VectorXf& raw_obs) {
//     std::lock_guard<std::mutex> lock(register_mutex_);
//     return tasks_[current_task_]->getAction(self_obs, raw_obs);
// }

// std::string BaseController::getVisualization(const Eigen::VectorXf& self_obs, const Eigen::VectorXf& raw_obs, const Eigen::VectorXf& action) {
//     std::lock_guard<std::mutex> lock(register_mutex_);
//     return tasks_[current_task_]->getVisualization(self_obs, raw_obs, action);
// }


// void BaseController::startKeyboardListener() {
//     keyboard_thread_running_ = true;
//     keyboard_thread_ = std::thread(&BaseController::keyboardLoop, this);
// }

// void BaseController::stopKeyboardListener() {
//     keyboard_thread_running_ = false;
//     if (keyboard_thread_.joinable()) keyboard_thread_.join();
// }

// void BaseController::keyboardLoop() {
//     while (keyboard_thread_running_) {
//         char key;
//         std::cin >> key;  // 🔁 替换为 SDL2 或 termios 更实时的监听方式
//         bool matched = false;

//         for (const auto& [name, k] : keyboard_mapping_) {
//             if (k == key) {
//                 std::lock_guard<std::mutex> lock(register_mutex_);
//                 current_task_ = name;
//                 tasks_[current_task_]->reset();
//                 matched = true;
//                 break;
//             }
//         }

//         if (!matched && key == 'v') {
//             std::lock_guard<std::mutex> lock(register_mutex_);
//             auto it = std::find(task_list_.begin(), task_list_.end(), current_task_);
//             if (++it == task_list_.end()) it = task_list_.begin();
//             current_task_ = *it;
//             tasks_[current_task_]->reset();
//         } else {
//             std::lock_guard<std::mutex> lock(register_mutex_);
//             tasks_[current_task_]->handleKey(key);
//         }
//     }
//
// }

