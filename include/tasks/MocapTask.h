// tasks/MocapTask.h
#pragma once
#include "tasks/BaseTask.h"
#include "tasks/TaskFactory.h"

struct MocapTaskCfg : public BaseTaskCfg {
    std::string policy_path = std::string(PROJECT_SOURCE_DIR) + "/resources/policies/g1/cmdTask.pt";
    std::string engine_path = std::string(PROJECT_SOURCE_DIR) + "/resources/policies/g1/cmdTask.engine";
    
    // 模型参数
    int num_obs = 96;         // 93 self_obs + 3 command
    int num_actions = 29;
    int num_hidden = 2883;    // e.g., 32 x 93 + 3 - 96

    // 指令限制与缩放因子
    Eigen::Vector3f max_cmd = {0.8f, 0.5f, 1.57f};
    Eigen::Vector3f obs_scale = {2.0f, 2.0f, 0.25f};

    float obs_scale_heading = 0.5f;
  
     // 远程设置（如果需要结构化建议换成 struct 或 json 配置）
    std::string remote_host = "192.168.123.111";
    int remote_port = 7003;

    // 轨迹采样设置
    int num_samples = 9;                // 1 current + 8 next
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

class MocapTask : public BaseTask {
public:
    MocapTask(std::shared_ptr<const BaseRobotConfig> cfg,
              torch::Device device,
              const std::string& inference_engine_type,
              const std::string& precision);
    void resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) override;
    void resolveObservation(const CustomTypes::RobotData& robotData) override;

private:
    MocapTaskCfg task_cfg_;
    float obs_scale_heading_;
    std::vector<int> track_keypoints_indices_;
    Eigen::VectorXf mask_;
};


// Register Task
namespace {
bool registered = []() {
    TaskFactory::registerTask("MocapTask", 
        [](std::shared_ptr<const BaseRobotConfig> cfg, 
           torch::Device device,
           const std::string& inference_engine_type,
           const std::string& precision) {
            return std::make_shared<MocapTask>(cfg, device, inference_engine_type, precision);
        });
    return true;
}();
}