#pragma once
#include "controller/BasePolicyWrapper.h"

class EmanPolicyWrapper : public BasePolicyWrapper  {
public:
    EmanPolicyWrapper(std::shared_ptr<const BaseRobotConfig> cfg, 
                      torch::Device device, 
                      const std::string& inference_engine_type, 
                      const std::string& precision);
    CustomTypes::Action getControlAction(const CustomTypes::RobotData &robotData) override;
    
private:
    void updateObservation(const CustomTypes::RobotData& robotData);
};
