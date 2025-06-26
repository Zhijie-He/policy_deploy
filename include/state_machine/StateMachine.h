#pragma once

#include <memory>
#include "config/BaseRobotConfig.h" 
#include "types/joystickTypes.h"
#include "types/system_defined.h"
#include "types/CustomTypes.h"
#include "utility/data_buffer.h"
#include "controller/BasePolicyWrapper.h"

class StateMachine {
public:
    explicit StateMachine(std::shared_ptr<const BaseRobotConfig> cfg, const std::string& config_name, torch::Device device);
    void run();
    virtual void step();
    virtual void stop(); 
    
    void setInputPtr(char* key, JoystickData* joy) {_jsStates = joy; _keyState = key;}
    std::shared_ptr<DataBuffer<robotStatus>> getRobotStatusBufferPtr() const {return _robotStatusBuffer;}
    std::shared_ptr<DataBuffer<jointCMD>> getJointCMDBufferPtr() const {return _jointCMDBuffer;}

protected:
    void getRawObs();
    void updateCommands();
    void packJointAction();
    
    std::shared_ptr<const BaseRobotConfig> cfg_; 
    char* _keyState = nullptr;
    JoystickData* _jsStates = nullptr;

    bool _isRunning = true;
    int _jointNum;
    float _policyDt;
    float yawTarg = 0;

    std::shared_ptr<DataBuffer<robotStatus>> _robotStatusBuffer;
    std::shared_ptr<DataBuffer<jointCMD>> _jointCMDBuffer;

    CustomTypes::Action robotAction;
    CustomTypes::RobotData robotData;

    int run_count=0;
    double run_sum_us=0;
    double run_sum_sq_us=0;
    std::unique_ptr<BasePolicyWrapper> _neuralCtrl;
};
