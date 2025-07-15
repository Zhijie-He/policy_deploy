// ===== include/state_machine/StateMachine.h =====
#pragma once

#include <vector>
#include <memory>
#include "config/BaseRobotConfig.h" 
#include "types/joystickTypes.h"
#include "types/system_defined.h"
#include "types/CustomTypes.h"
#include "utility/data_buffer.h"
#include "tasks/BaseTask.h"
#include "tasks/TaskFactory.h"

class StateMachine {
public:
    StateMachine(std::shared_ptr<const BaseRobotConfig> cfg, 
                 torch::Device device,
                 const std::vector<std::pair<std::string, char>>& registers,
                 const std::string& hands_type,
                 const std::string& inference_engine_type = "libtorch", 
                 const std::string& precision = "fp32");
                 
    void createTasks(const std::vector<std::pair<std::string, char>>& registers, 
                     torch::Device device, 
                     const std::string& inference_engine_type, 
                     const std::string& precision);
    void handleKeyboardInput(char c);
    void step();
    
    void setInputPtr(char* key, JoystickData* joy) {_jsStates = joy; _keyState = key;}
    std::shared_ptr<DataBuffer<robotStatus>> getRobotStatusBufferPtr() const {return _robotStatusBuffer;}
    std::shared_ptr<DataBuffer<jointCMD>> getJointCMDBufferPtr() const {return _jointCMDBuffer;}
    const Eigen::MatrixXf& getTaskVisualization();
    
protected:
    void getRawObs();
    void packJointAction();
    
    std::shared_ptr<const BaseRobotConfig> cfg_; 
    char* _keyState = nullptr;
    std::mutex key_state_lock_;
    JoystickData* _jsStates = nullptr;

    int _jointNum;
    int _handsNum;
    std::string hands_type_;
    std::shared_ptr<DataBuffer<robotStatus>> _robotStatusBuffer;
    std::shared_ptr<DataBuffer<jointCMD>> _jointCMDBuffer;

    CustomTypes::Action robotAction;
    CustomTypes::RobotData robotData;
    
    std::string current_task_;
    std::vector<std::string> task_name_list_;
    std::unordered_map<std::string, std::shared_ptr<BaseTask>> tasks_;
    std::unordered_map<std::string, char> task_key_map_;
    std::mutex task_lock_;
};
