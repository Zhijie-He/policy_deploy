#pragma once

#include <memory>
#include "core/RobotConfig.h" 
#include "types/joystickTypes.h"
#include "types/system_defined.h"
#include "types/CustomTypes.h"


class StateMachine {
public:
    explicit StateMachine(std::shared_ptr<const RobotConfig> cfg);
    void run();
    virtual void step() {}
    virtual void stop(); 
    
    void setInputPtr(char* key, JoystickData* joy) {_jsStates = joy; _keyState = key;}
    GyroData* getGyroPtr() {return _gyroStates;}
    jointStateData* getMotorStatePtr() {return _motorStates;}
    jointTargetData* getMotorTargetPtr() {return _motorTargets;}
    
protected:
    void parseRobotStates();
    void updateCommands();
    void packRobotAction();
    
    std::shared_ptr<const RobotConfig> cfg_; 
    char* _keyState = nullptr;
    JoystickData* _jsStates = nullptr;

    bool isFiltersEnable_ = false;
    bool _isRunning = true;
    int _jointNum = 28;
    float _policyDt = 0.001; // 表示策略（如神经网络、状态机）做决策的频率；
    float yawTarg = 0;

    jointStateData* _motorStates = nullptr;
    jointTargetData* _motorTargets = nullptr;
    GyroData* _gyroStates = nullptr;
    CustomTypes::State robotState;
    CustomTypes::Action robotAction;
};
