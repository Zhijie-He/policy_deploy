#include "state_machine/StateMachine.h"
#include "utility/logger.h"

StateMachine::StateMachine(const YAML::Node& cfg):
  _jointNum(cfg["joint_num"].as<int>()),
  _policyDt(cfg["policy_dt"].as<float>()){
    
  /// Joint order: waist - Right legs - Left legs - Neck - Right Arm - Left Arm
  _jointStateBuf = std::make_shared<motor_state_data>();
  _jointCmdBuf = std::make_shared<motor_cmd_data>();
  _gyroStateBuf = std::make_shared<GyroData>();

  robotState = CustomTypes::zeroState(_jointNum);
  robotAction = CustomTypes::zeroAction(_jointNum);

  // initFilters(cfg["filter"], _jointNum);
}

void StateMachine::updateAction() {

}


