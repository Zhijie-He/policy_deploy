#pragma once

#include <memory>
#include "config/BaseRobotConfig.h" 
#include "types/joystickTypes.h"
#include "types/system_defined.h"
#include "types/CustomTypes.h"
#include "hardware/listener.h"
#include "utility/data_buffer.h"
#include "policy_wrapper/BasePolicyWrapper.h"

class StateMachine {
public:
    explicit StateMachine(std::shared_ptr<const BaseRobotConfig> cfg, 
                          const std::string& config_name, 
                          torch::Device device,
                          const std::string& inference_engine_type = "libtorch", 
                          const std::string& precision = "fp32");
    void run();
    virtual void step();
    virtual void stop(); 
    
    void setInputPtr(std::shared_ptr<Listener> listener, JoystickData* joy) {listenerPtr_ = listener; _jsStates = joy;}
    std::shared_ptr<DataBuffer<robotStatus>> getRobotStatusBufferPtr() const {return _robotStatusBuffer;}
    std::shared_ptr<DataBuffer<jointCMD>> getJointCMDBufferPtr() const {return _jointCMDBuffer;}

protected:
    void getRawObs();
    void updateCommands();
    void packJointAction();
    
    std::shared_ptr<const BaseRobotConfig> cfg_; 
    
    std::shared_ptr<Listener> listenerPtr_;
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
