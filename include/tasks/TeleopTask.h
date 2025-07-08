// tasks/TeleopTask.h
#pragma once
#include "tasks/BaseTask.h"
#include "tasks/TaskFactory.h"
#include "utility/json.hpp"

using json = nlohmann::json;

struct TeleopTaskCfg : public BaseTaskCfg {
    std::string policy_path = std::string(PROJECT_SOURCE_DIR) + "/resources/policies/g1/teleopTask.pt";
    std::string engine_path = std::string(PROJECT_SOURCE_DIR) + "/resources/policies/g1/teleopTask.engine";
    std::string motion_json_file = std::string(PROJECT_SOURCE_DIR) + "/resources/sample_data/teleop_motion_lib_cache.json";

    int num_obs = 934;       // 93 + 91 + 8x90 + 30
    int num_hidden = 5704;   //  5704  = 31 x(91 + 93)
    int num_actions = 29;    

    float obs_scale_heading = 0.5f;
    int num_motions = 2;
    int num_samples = 1 + 8;
    float sample_timestep_inv = 30.0f;

    std::vector<std::string> track_keypoints_names = {
        "pelvis",
        "left_knee_link", "left_ankle_roll_link", "right_knee_link", "right_ankle_roll_link",
        "left_elbow_link","left_wrist_yaw_link", "right_elbow_link","right_wrist_yaw_link"
    };

    std::vector<std::string> BODY_NAMES = {
        "pelvis",
        "left_hip_pitch_link","right_hip_pitch_link","waist_yaw_link",
        "left_hip_roll_link", "right_hip_roll_link", "waist_roll_link",
        "left_hip_yaw_link", "right_hip_yaw_link", "torso_link",
        "left_knee_link", "right_knee_link", "left_shoulder_pitch_link", "right_shoulder_pitch_link",
        "left_ankle_pitch_link", "right_ankle_pitch_link", "left_shoulder_roll_link", "right_shoulder_roll_link",
        "left_ankle_roll_link", "right_ankle_roll_link", "left_shoulder_yaw_link", "right_shoulder_yaw_link",
        "left_elbow_link", "right_elbow_link",
        "left_wrist_roll_link", "right_wrist_roll_link",
        "left_wrist_pitch_link", "right_wrist_pitch_link",
        "left_wrist_yaw_link", "right_wrist_yaw_link"
    };

    std::string getPolicyPath() const override{
        return policy_path;
    }

    std::string getEnginePath() const override{
        return engine_path;
    }
    
    int getNumActions() const override{
        return num_actions;
    }

    int getNumObs() const override{
        return num_obs;
    }

    int getNumHidden() const override{
        return num_hidden;
    }
};


class TeleopTask : public BaseTask {
public:
    TeleopTask(std::shared_ptr<const BaseRobotConfig> cfg,
               torch::Device device,
               const std::string& inference_engine_type,
               const std::string& precision);

    void resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) override;
    void resolveObservation(const CustomTypes::RobotData& robotData) override;
    void loadMotion();
    void reset() override;

private:
    TeleopTaskCfg task_cfg_;
    float obs_scale_heading_;
    std::vector<int> track_keypoints_indices_;
    Eigen::VectorXf mask_;
    int motion_id_;
    int count_offset_;
    std::vector<int> motion_lib_cache_len_;
    json motion_lib_cache_;
};


// Register Task
namespace {
bool registered = []() {
    TaskFactory::registerTask("TeleopTask", 
        [](std::shared_ptr<const BaseRobotConfig> cfg, 
           torch::Device device,
           const std::string& inference_engine_type,
           const std::string& precision) {
            return std::make_shared<TeleopTask>(cfg, device, inference_engine_type, precision);
        });
    return true;
}();
}

