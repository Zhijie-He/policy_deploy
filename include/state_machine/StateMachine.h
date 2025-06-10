#pragma once

#include <memory>
#include "core/BaseRobotConfig.h" 
#include "types/joystickTypes.h"
#include "types/system_defined.h"
#include "types/CustomTypes.h"
#include "utility/data_buffer.h"

class StateMachine {
public:
    explicit StateMachine(std::shared_ptr<const BaseRobotConfig> cfg);
    void run();
    virtual void step() {}
    virtual void stop(); 
    
    void setInputPtr(char* key, JoystickData* joy) {_jsStates = joy; _keyState = key;}
    std::shared_ptr<DataBuffer<robotStatus>> getRobotStatusBufferPtr() const {return _robotStatusBuffer;}
    std::shared_ptr<DataBuffer<jointCMD>> getJointCMDBufferPtr() const {return _jointCMDBuffer;}

    protected:
    void parseRobotData();
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
};
