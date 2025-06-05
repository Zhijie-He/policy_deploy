#pragma once

#include <memory>
#include "core/BaseRobotConfig.h" 
#include "types/joystickTypes.h"
#include "types/system_defined.h"
#include "types/CustomTypes.h"


class StateMachine {
public:
    explicit StateMachine(std::shared_ptr<const BaseRobotConfig> cfg);
    void run();
    virtual void step() {}
    virtual void stop(); 
    
    void setInputPtr(char* key, JoystickData* joy) {_jsStates = joy; _keyState = key;}

    std::shared_ptr<robotStatus> getRobotStatusPtr() { return _robotStatus; }
    std::shared_ptr<jointCMD> getJointCMDPtr() { return _jointCMD; }

    protected:
    void parseRobotData();
    void updateCommands();
    void packJointAction();
    
    std::shared_ptr<const BaseRobotConfig> cfg_; 
    char* _keyState = nullptr;
    JoystickData* _jsStates = nullptr;

    bool _isRunning = true;
    int _jointNum;
    float _policyDt; // 表示策略（如神经网络、状态机）做决策的频率；
    float yawTarg = 0;
    
    std::shared_ptr<robotStatus> _robotStatus;
    std::shared_ptr<jointCMD> _jointCMD;
    
    CustomTypes::Action robotAction;
    CustomTypes::RobotData robotData;
};
