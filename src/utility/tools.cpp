#include "utility/tools.h"

namespace tools {
    std::shared_ptr<BaseRobotConfig> loadConfig(const std::string& config_name) {
        const std::string config_path = std::string(PROJECT_SOURCE_DIR) + "/config/" + config_name + ".yaml";
        if (config_name == "g1_eman")
            return std::make_shared<EmanRobotConfig>(config_path);
        else
            return std::make_shared<UnitreeRobotConfig>(config_path);
    }

    std::unique_ptr<NeuralController> loadPolicyWrapper(const std::string& config_name, std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device) {
        if (config_name == "g1_eman")
            return std::make_unique<EmanPolicyWrapper>(cfg, torch::Device device); 
        else
            return std::make_unique<UnitreePolicyWrapper>(cfg, torch::Device device);
    }

    Eigen::VectorXf pd_control(const Eigen::VectorXf& target_q,
                                  const Eigen::VectorXf& q,
                                  const Eigen::VectorXf& kp,
                                  const Eigen::VectorXf& target_dq,
                                  const Eigen::VectorXf& dq,
                                  const Eigen::VectorXf& kd) {
    return (target_q - q).cwiseProduct(kp) + (target_dq - dq).cwiseProduct(kd);
    }

    void checkMujucoVersion(){
        if (mjVERSION_HEADER != mj_version()) {  // Mujoco版本检查
            FRC_ERROR("[checkMujucoVersion] MuJoCo header and library version mismatch!");
            throw std::runtime_error("MuJoCo header and library version mismatch!");
        }
        FRC_INFO("[checkMujucoVersion] Using MuJoCo("<< mj_version() << ")!");
    }

    torch::Device getDefaultDevice() {
        #if defined(__APPLE__)
            FRC_INFO("[getDefaultDevice] Running on macOS. Using CPU.");
            return torch::kCPU;
        #elif defined(_WIN32) || defined(__linux__)
            if (torch::cuda::is_available()) {
                FRC_INFO("[getDefaultDevice] CUDA available. Using GPU.");
                return torch::kCUDA;
            } else {
                FRC_INFO("[getDefaultDevice] CUDA not available. Using CPU.");
                return torch::kCPU;
            }
        #else
            FRC_WARN("[getDefaultDevice] Unknown platform. Defaulting to CPU.");
            return torch::kCPU;
        #endif
    }
}

