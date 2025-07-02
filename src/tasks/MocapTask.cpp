// tasks/MocapTask.cpp
#include "tasks/MocapTask.h"
#include "utility/logger.h"
#include "utility/tools.h"

MocapTask::MocapTask(std::shared_ptr<const BaseRobotConfig> cfg,
                     torch::Device device,
                     const std::string& inference_engine_type,
                     const std::string& precision)
      : BaseTask(cfg, std::make_shared<MocapTaskCfg>(), device,  inference_engine_type, precision),
        task_cfg_() 
{
    FRC_INFO("[MocapTask.Const] Created!");
}

void MocapTask::resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) {
    FRC_INFO("[MocapTask.resolveKeyboardInput] Key pressed: " << key);
}

void MocapTask::resolveObservation(const CustomTypes::RobotData &raw_obs) {
    updateObservation(raw_obs);
}
