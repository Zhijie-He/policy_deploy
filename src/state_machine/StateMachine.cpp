#include <thread>
#include <chrono>
#include "state_machine/StateMachine.h"
#include "utility/timer.h"
#include "utility/tools.h"
#include "utility/logger.h"

StateMachine::StateMachine(std::shared_ptr<const BaseRobotConfig> cfg, 
                           torch::Device device,
                           const std::vector<std::pair<std::string, char>>& registers,
                           const std::string& inference_engine_type,
                           const std::string& precision)
                           : cfg_(cfg),
                           _jointNum(cfg->num_actions)
{
  // 1. 初始化底层设备数据结构
  _robotStatusBuffer = std::make_shared<DataBuffer<robotStatus>>();
  _jointCMDBuffer = std::make_shared<DataBuffer<jointCMD>>();

  // 2. 初始化机器人状态和输出动作向量为零
  robotAction = CustomTypes::zeroAction(_jointNum);
  robotData = CustomTypes::zeroData(_jointNum);

  // 3. 创建任务
  createTasks(registers, device, inference_engine_type, precision);
}

void StateMachine::createTasks(const std::vector<std::pair<std::string, char>>& registers, 
                               torch::Device device, 
                               const std::string& inference_engine_type, 
                               const std::string& precision)
{
  // check if task_name is available and create tasks
  for (const auto& [task_name, key] : registers) {
    if (!TaskFactory::exists(task_name)) {
      FRC_ERROR("[StateMachine.createTasks] Invalid task name: " << task_name);
      auto available = TaskFactory::getAvailableTaskNames();
      std::ostringstream oss;
      oss << "[StateMachine.createTasks] Available task names: ";
      for (size_t i = 0; i < available.size(); ++i) {
          oss << available[i];
          if (i != available.size() - 1) oss << " | ";
      }
      FRC_ERROR(oss.str());
      throw std::runtime_error("Invalid task name: " + task_name);
    }
    auto task = TaskFactory::create(task_name, cfg_, device, inference_engine_type, precision);
    tasks_[task_name] = task;
    task_key_map_[task_name] = key;
    task_name_list_.push_back(task_name);
  }
  
  // default activate the first task
  if (!task_name_list_.empty()) {
      current_task_ = task_name_list_.front();  
      FRC_INFO("[StateMachine.createTasks] Current active task: " << current_task_);
  } else {
      FRC_ERROR("[StateMachine.createTasks] No tasks registered in StateMachine");
      throw std::runtime_error("No  tasks registered in StateMachine.");
  }

  // print all available tasks
  std::ostringstream oss;
  oss << "[StateMachine.createTasks] Available Tasks: ";
  for (const auto& r : task_name_list_) oss << r << " ";
  FRC_INFO(oss.str());
}

void StateMachine::handleKeyboardInput(char c) {
  // 1. Switch to specific task by key
  for (const auto& [task_name, key] : task_key_map_) {
    if (c == key) {
      {
        std::lock_guard<std::mutex> lock(task_lock_);
        current_task_ = task_name;
        FRC_INFO("[StateMachine.handleKeyboardInput] Switched to task: " << current_task_);
        tasks_[current_task_]->reset();
      }
      {
        std::lock_guard<std::mutex> key_lock(key_state_lock_);
        *_keyState = '\0';
      }
      return;
    }
  }

  // 2. Cycle to next task with 'v'
  if (c == 'v') {
    std::lock_guard<std::mutex> lock(task_lock_);
    auto it = std::find(task_name_list_.begin(), task_name_list_.end(), current_task_);
    if (it != task_name_list_.end()) {
      size_t idx = (std::distance(task_name_list_.begin(), it) + 1) % task_name_list_.size();
      current_task_ = task_name_list_[idx];
      FRC_INFO("[StateMachine.handleKeyboardInput] Cycled to task: " << current_task_);
      tasks_[current_task_]->reset();
    }
    {
      std::lock_guard<std::mutex> key_lock(key_state_lock_);
      *_keyState = '\0';
    }
    return;
  }
}

void StateMachine::step() {
  getRawObs();
  {
    std::lock_guard<std::mutex> lock(task_lock_);
    char key = '\0';

    {
      std::lock_guard<std::mutex> key_lock(key_state_lock_);
      if (*_keyState != '\0') {
        key = *_keyState;
        *_keyState = '\0'; 
      }
    }

    if (key != '\0') {
      tasks_[current_task_]->resolveKeyboardInput(key, robotData);
    }

    robotAction = tasks_[current_task_]->getAction(robotData);
  }
  packJointAction();
}

void StateMachine::getRawObs() {
  assert(_robotStatusBuffer != nullptr);
  auto status_ptr = _robotStatusBuffer->GetData();
  while (!status_ptr) { // 这里存在有可能jointCMD还没有设定值 但是这里在读取 所以要等待
    FRC_INFO("[StateMachine.getRawObs] Waiting for robotStatusBuffer...");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    status_ptr = _robotStatusBuffer->GetData();  // 重新尝试获取
  }

  const auto& status = *status_ptr;  // 解引用拿结构体  推荐：零拷贝 + 明确只读引用
  robotData.timestamp = status.data.timestamp;

  robotData.root_xyz = Eigen::Map<const Eigen::Vector3f>(status.data.position);
  robotData.root_rot     = Eigen::Map<const Eigen::Vector4f>(status.data.position + 3);
  robotData.joint_pos = Eigen::Map<const Eigen::VectorXf>(status.data.position + 7, _jointNum);

  robotData.root_vel = Eigen::Map<const Eigen::Vector3f>(status.data.velocity);
  robotData.root_ang_vel    = Eigen::Map<const Eigen::Vector3f>(status.data.velocity + 3);
  robotData.joint_vel = Eigen::Map<const Eigen::VectorXf>(status.data.velocity + 6, _jointNum);

  robotData.joint_torques = Eigen::Map<const Eigen::VectorXf>(status.data.jointTorques, _jointNum);
}

void StateMachine::packJointAction(){
  assert(_jointCMDBuffer != nullptr);
  jointCMD cmd;
  cmd.data.timestamp = robotAction.timestamp;
  memcpy(cmd.data.position, robotAction.motorPosition.data(), _jointNum * sizeof(float));
  memcpy(cmd.data.velocity, robotAction.motorVelocity.data(), _jointNum * sizeof(float));
  memcpy(cmd.data.kp, robotAction.kP.data(), _jointNum * sizeof(float));
  memcpy(cmd.data.kd, robotAction.kD.data(), _jointNum * sizeof(float));
  _jointCMDBuffer->SetData(cmd);
}

